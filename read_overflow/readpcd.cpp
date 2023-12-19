#include <iostream>          //标准c++库输入输出相关头文件
#include <pcl/io/pcd_io.h>   // pcd读写相关头文件
#include <pcl/point_types.h> // pcl中支持的点类型头文件
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
// 定义点云格式，具体见下章
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using namespace std;
using namespace cv;
bool overflow_status = 0; //暂时发生溢锅了就认为一直溢锅

void IRThread()
{
    std::string ir_pattern_jpg = "../ir/*.png";
    std::vector<cv::String> ir_ply_files;
    cv::glob(ir_pattern_jpg, ir_ply_files);

    for (int ii = 0; ii < ir_ply_files.size(); ii++)
    {
        PCL_ERROR("ir thread\n");
        cout << ir_ply_files[ii] << endl;
        Mat ir_image = imread(ir_ply_files[ii], CV_LOAD_IMAGE_COLOR);
        if (ir_image.empty())
        {
            cout << "Could not open or find the image" << endl;
            //return -1;
        }
        if (overflow_status == 1)
        {
            putText(ir_image, "OverFlow", cv::Point(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        }
        namedWindow("IR", 0);
        resizeWindow("IR", 640, 480);
        imshow("IR", ir_image);
        waitKey(120);

        //usleep(200000);
    }
}
void getPassTroughFilterData(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, const std::string m_coordinate, const double m_limit_min, const double m_limit_max)
{
    std::cout << "CDataFilter: PointCloud before filtering has: "
              << cloud_in->points.size()
              << " data points." << std::endl;

    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(cloud_in);
    // 限制坐标轴
    pass.setFilterFieldName(m_coordinate);
    // 设置滤波范围
    pass.setFilterLimits(m_limit_min, m_limit_max);
    pass.filter(*cloud_out);

    std::cout << "CDataFilter: PointCloud after PassTrough filtering has: "
              << cloud_out->points.size()
              << " data points." << std::endl;
}

int main(int argc, char **argv)
{
    // 定义点云
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_filter(new PointCloud);
    PointCloud::Ptr cloud_out(new PointCloud);
    PointCloud::Ptr cloud_segmentation_x(new PointCloud);
    PointCloud::Ptr cloud_segmentation_y(new PointCloud);
    PointCloud::Ptr cloud_segmentation_z(new PointCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<PointCloud::Ptr> cloud_vector;
    std::vector<double> z_ave_vector;               //均值
    std::vector<double> z_k_vector;                 //斜率
    std::vector<double> z_ave_filter_vector;        //滤波
    std::vector<double> z_ave_filter_vector_two;    //第二次滤波
    std::vector<double> z_ave_filter_change_vector; //滤波后变化
    double filter_change = 0.0;

    double x_range_max = 0.0;
    double x_range_min = 0.0;
    double y_range_max = 0.0;
    double y_range_min = 0.0;
    //读取点云，失败返回-1
    //	if (pcl::io::loadPCDFile<PointT>("0.pcd", *cloud_in) == -1)
    //	{
    //		PCL_ERROR("couldn't read file\n");
    //		return (-1);
    //	}
    string argv_ir = "ir";
    if (argc > 1)
    {
        if (argv[1] == argv_ir)
        {
            boost::thread *IRLoopThread = new boost::thread(boost::bind(&IRThread));
        }
    }

    std::string pattern_jpg = "../data/*.pcd";
    std::vector<cv::String> ply_files;
    cv::glob(pattern_jpg, ply_files);
    pcl::visualization::CloudViewer viewer("visual"); //框的名字
    for (int ii = 0; ii < ply_files.size(); ii++)
    {
        pcl::io::loadPCDFile(ply_files[ii], *cloud_in);
        //1. 滤波1
        std::cout << "before filter size: " << cloud_in->points.size() << endl;
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud(cloud_in);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_filter);
        std::cout << "after filter size: " << cloud_filter->points.size() << endl;
        //滤波2
        getPassTroughFilterData(cloud_filter, cloud_segmentation_x, "x", 0, 0.55);
        getPassTroughFilterData(cloud_segmentation_x, cloud_segmentation_y, "y", -0.35, 0.1);
        getPassTroughFilterData(cloud_segmentation_y, cloud_segmentation_z, "z", 0, 0.5);
        //pcl::io::savePCDFileBinary("./pot.pcd", *cloud_segmentation_z);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_segmentation_z, *cloud_segment);
        //2. 分割
        double m_clustertolerance = 0.02; //0.02能分开，0.04没分开，0。01太细了不能用，0。03能分开分出了16个，0.02分出24个（用0.02分的细一点）
        int m_minclustersize = 40;
        int m_maxclustersize = 25000;

        std::vector<pcl::PointIndices> cluster_indices;
        search_tree->setInputCloud(cloud_segment);
        // Euclidean Clustering

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece; //欧式聚类对象
        ece.setClusterTolerance(m_clustertolerance);        //设置近邻搜索的搜索半径为2cm
        ece.setMinClusterSize(m_minclustersize);            //设置一个聚类需要的最少的点数目为100
        ece.setMaxClusterSize(m_maxclustersize);            //设置一个聚类需要的最大点数目为25000
        ece.setSearchMethod(search_tree);                   //设置点云的搜索机制
        ece.setInputCloud(cloud_segment);
        ece.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中

        pcl::copyPointCloud(*cloud_segment, *cloud_out);
        int count = 0;
        //对分割后的点云进行保存，放入向量
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
             it != cluster_indices.end();
             ++it)
        {
            PointCloud::Ptr cloud_cluster(new PointCloud);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {

                cloud_cluster->points.push_back(cloud_out->points[*pit]);
            }

            cout << count << endl;
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            cloud_vector.push_back(cloud_cluster); //和count计数多少没关系，分离后都添加到向量里
            stringstream view_out;
            std::string view_zero;
            if (count < 10)
            {
                view_zero = "0000";
            }
            else if (count >= 10 && count < 100)
            {
                view_zero = "000";
            }
            else
            {
                view_zero = "00";
            }
            view_out << "./" << view_zero << count << ".pcd";
            pcl::io::savePCDFileBinary(view_out.str(), *cloud_cluster);
            if (count < 20)
                count++;
        }

        //3.向量中找锅点云
        short pot_count = 0;
        PointCloud::Ptr pot_cloud(new PointCloud);
        for (int i = 0; i < cloud_vector.size(); i++)
        {
            std::cout << "size: " << cloud_vector[i]->points.size() << " x: " << cloud_vector[i]->points[10].x << " y: " << cloud_vector[i]->points[10].y << " z: " << cloud_vector[i]->points[10].z << std::endl;
            if (cloud_vector[i]->points.size() > 200 && cloud_vector[i]->points.size() < 1000)
            {
                //            if(cloud_vector[i]->points[10].x<0.4 && cloud_vector[i]->points[10].x>-0.5)
                //            {
                //                if(cloud_vector[i]->points[10].y<0.1 && cloud_vector[i]->points[10].y>-0.4)
                //                {
                //                    if(cloud_vector[i]->points[10].z>0 && cloud_vector[i]->points[10].z<0.55)
                //                    {
                pcl::copyPointCloud(*cloud_vector[i], *pot_cloud);
                cout << pot_count << endl;
                pot_count++;
                if (pot_count > 1)
                {
                    PCL_ERROR("pot cloud recognize has problem\n");
                    //return (-1);
                }
                //                    }
                //                }
                //            }
            }
        }
        if (pot_count == 0)
        {
            PCL_ERROR("no pot cloud\n");
            cout << ii << endl;
            continue;
            //return (-1);
        }
        //pcl::io::savePCDFileBinary("./pot.pcd",*pot_cloud);
        //4。找锅最左侧点（后续不同灶心再改）
        double x_min = 10;
        int pot_min_index = 0;
        for (int i = 0; i < pot_cloud->points.size(); i++)
        {
            if (pot_cloud->points[i].x < x_min)
            {
                x_min = pot_cloud->points[i].x;
                pot_min_index = i;
            }
        }
        cout << "x_min: " << x_min << endl;
        cout << "x: " << pot_cloud->points[pot_min_index].x << " y: " << pot_cloud->points[pot_min_index].y << " z: " << pot_cloud->points[pot_min_index].z << endl;

        //4.1 最小x应该是固定值，之后锅识别会变动,范围只确定一次，具体范围根据数据集确定
        if (ii == 0) //后续改成i,只需要计算一次？
        {
            x_range_max = pot_cloud->points[pot_min_index].x - 0.01;
            x_range_min = pot_cloud->points[pot_min_index].x - 0.02;
            //            x_range_max = pot_cloud->points[pot_min_index].x;
            //            x_range_min = pot_cloud->points[pot_min_index].x-0.01;
            y_range_max = pot_cloud->points[pot_min_index].y;
            y_range_min = pot_cloud->points[pot_min_index].y - 0.06; //0.03
        }

        //5.对原始点云进行上色 求高度和
        double pot_sum_z = 0.0;
        double pot_count_z = 0.0;
        for (int i = 0; i < cloud_filter->points.size(); i++)
        {
            cloud_filter->points[i].r = 255;
            cloud_filter->points[i].g = 255;
            cloud_filter->points[i].b = 255;
            if (cloud_filter->points[i].x < x_range_max && cloud_filter->points[i].x > x_range_min && cloud_filter->points[i].y < y_range_max && cloud_filter->points[i].y > y_range_min)
            {
                cloud_filter->points[i].r = 255;
                //cloud_filter->points[i].r = 0;
                cloud_filter->points[i].g = 0;
                cloud_filter->points[i].b = 0;
                pot_sum_z = pot_sum_z + cloud_filter->points[i].z;
                pot_count_z++;
            }
            if (cloud_filter->points[i].x < 0.25 && cloud_filter->points[i].x > 0.23 && cloud_filter->points[i].y < -0.20 && cloud_filter->points[i].y > -0.25)
            {
                cloud_filter->points[i].r = 0;
                //cloud_filter->points[i].r = 0;
                cloud_filter->points[i].g = 255;
                cloud_filter->points[i].b = 255;
                //pot_sum_z = pot_sum_z + cloud_filter->points[i].z;
                //pot_count_z++;
            }
        }
        pcl::io::savePCDFileBinary("./test.pcd", *cloud_filter);
        //存在点云有点，识别的区域没点的情况
        if (pot_count_z == 0)
        {
            continue;
        }
        double pot_ave_z = pot_sum_z / pot_count_z;
        pot_ave_z = (double)round(pot_ave_z * 100) / 100; //保留两位小数
        z_ave_vector.push_back(pot_ave_z);

        std::cout << "sum_z: " << pot_sum_z << " count_z: " << pot_count_z << " ave_z: " << pot_ave_z << std::endl;

        //for(int i = 0;i<z_ave_vector.size();i++)
        //{
        //    cout<<z_ave_vector[i]<<" ";
        //}
        //cout<<endl;

        //z值均值滤波
        double sum_filter = 0.0;
        double ave_filter = 0.0;
        int filter_scale = 9;
        if (z_ave_vector.size() < filter_scale)
        {
            for (int i = 0; i < z_ave_vector.size(); i++)
            {
                sum_filter = sum_filter + z_ave_vector.at(i);
            }
            ave_filter = sum_filter / z_ave_vector.size();
        }
        else
        {
            for (int i = (z_ave_vector.size() - filter_scale); i < z_ave_vector.size(); i++)
            {
                sum_filter = sum_filter + z_ave_vector.at(i);
            }
            ave_filter = sum_filter / filter_scale;
        }
        ave_filter = (double)round(ave_filter * 100) / 100;
        z_ave_filter_vector.push_back(ave_filter);

        for (int i = 0; i < z_ave_filter_vector.size(); i++)
        {
            cout << z_ave_filter_vector[i] << " ";
        }
        cout << endl;

        //第二次滤波
        double sum_filter_two = 0.0;
        double ave_filter_two = 0.0;
        if (z_ave_filter_vector.size() < filter_scale)
        {
            for (int i = 0; i < z_ave_filter_vector.size(); i++)
            {
                sum_filter_two = sum_filter_two + z_ave_filter_vector.at(i);
            }
            ave_filter_two = sum_filter_two / z_ave_filter_vector.size();
        }
        else
        {
            for (int i = (z_ave_filter_vector.size() - filter_scale); i < z_ave_filter_vector.size(); i++)
            {
                sum_filter_two = sum_filter_two + z_ave_filter_vector.at(i);
            }
            ave_filter_two = sum_filter_two / filter_scale;
        }
        ave_filter_two = (double)round(ave_filter_two * 100) / 100;

        z_ave_filter_vector_two.push_back(ave_filter_two);

        for (int i = 0; i < z_ave_filter_vector_two.size(); i++)
        {
            cout << z_ave_filter_vector_two[i] << " ";
        }
        cout << endl;

        //        //滤波后变化  没效果
        //        if(z_ave_filter_vector.size() >1)
        //        {
        //            filter_change += (z_ave_filter_vector.back() - z_ave_filter_vector.at(z_ave_filter_vector.size()-2)); //最后两个数之差（累积）
        //        }
        //
        //        z_ave_filter_change_vector.push_back(filter_change);
        //
        //        for(int i = 0;i<z_ave_filter_change_vector.size();i++)
        //        {
        //            cout<<z_ave_filter_change_vector[i]<<" ";
        //        }
        //        cout<<endl;

        //计算滤波后向量均值
        double z_ave_filter_two = 0.0;
        double z_sum_filter_two = 0.0;
        for (int i = 0; i < z_ave_filter_vector_two.size(); i++)
        {
            z_sum_filter_two += z_ave_filter_vector_two[i];
        }
        z_ave_filter_two = z_sum_filter_two / z_ave_filter_vector_two.size();
        cout << z_ave_filter_two << endl;
        //判断溢锅
        int ave_frame = 5;
        int overflow = 0;
        if (z_ave_filter_vector_two.size() >= ave_frame)
        {
            for (int i = 1; i <= ave_frame; i++)
            {
                //if(z_ave_filter_vector_two.at(z_ave_filter_vector_two.size()-i) <= -0.02)//连续5帧都<=-0.02
                if (z_ave_filter_vector_two.at(z_ave_filter_vector_two.size() - i) < (z_ave_filter_two - 0.02)) //连续5帧都<均值
                {
                    overflow++;
                }
            }
        }
        if (overflow == ave_frame)
        {
            overflow_status = 1;
        }
        if (overflow_status == 1)
        {
            PCL_ERROR("Overflow of the pot occurs\n");
            //点云上色，更直观显示
            for (int i = 0; i < cloud_filter->points.size(); i++)
            {
                if (cloud_filter->points[i].x < -0.5 && cloud_filter->points[i].x > -0.6 && cloud_filter->points[i].y < -0.3 && cloud_filter->points[i].y > -0.4)
                {
                    cloud_filter->points[i].r = 255;
                    cloud_filter->points[i].g = 0;
                    cloud_filter->points[i].b = 0;
                }
            }
        }
        //当前点斜率
        //        double pot_k_z = 0.0;
        //        if(z_ave_vector.size()>3)
        //        {
        //            pot_k_z = pot_ave_z - z_ave_vector.at(z_ave_vector.size()-3);//-2是倒数第2个，-3是倒数第三个
        //        }
        //        z_k_vector.push_back(pot_k_z);
        //
        //
        //        for(int i = 0;i<z_k_vector.size();i++)
        //        {
        //            cout<<z_k_vector[i]<<" ";
        //        }
        //        cout<<endl;

        usleep(100000);
        viewer.showCloud(cloud_filter);
        //vector要清空
        cloud_vector.clear();

        // if (ii>65)
        // if (ii>10)
        // {
        //     while(1)
        //     {
        //         if  (getchar() == 'a')
        //         {
        //             break;
        //         }
                
        //     }
        // }

        

    }

    return (0);
}

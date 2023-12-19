#include <iostream> //标准c++库输入输出相关头文件
#include <pcl/io/pcd_io.h> // pcd读写相关头文件
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
#define center_x 0.259029
#define center_y -0.240518
#define PI 3.1415926

void getPassTroughFilterData(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, const std::string m_coordinate, const double m_limit_min, const double m_limit_max)
{
    std::cout << "CDataFilter: PointCloud before filtering has: "
    	<< cloud_in->points.size ()
    	<< " data points." << std::endl;

    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (cloud_in);
    // 限制坐标轴
    pass.setFilterFieldName (m_coordinate);
    // 设置滤波范围
    pass.setFilterLimits (m_limit_min, m_limit_max);
    pass.filter (*cloud_out);

    std::cout << "CDataFilter: PointCloud after PassTrough filtering has: "
    	<< cloud_out->points.size ()
    	<< " data points." << std::endl;

}

int main(int argc, char** argv)
{
	// 定义点云
	PointCloud::Ptr cloud_in(new PointCloud);
	PointCloud::Ptr cloud_filter(new PointCloud);
    PointCloud::Ptr cloud_out (new PointCloud);
    PointCloud::Ptr cloud_segmentation_x(new PointCloud);
    PointCloud::Ptr cloud_segmentation_y(new PointCloud);
    PointCloud::Ptr cloud_segmentation_z(new PointCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<PointCloud::Ptr> cloud_vector;

    //读取点云，失败返回-1
//	if (pcl::io::loadPCDFile<PointT>("0.pcd", *cloud_in) == -1)
//	{
//		PCL_ERROR("couldn't read file\n");
//		return (-1);
//	}


    std::string pattern_jpg = "../data/*.pcd";
    std::vector<cv::String> ply_files;
    cv::glob(pattern_jpg, ply_files);
    pcl::visualization::CloudViewer viewer("visual");//框的名字
    for(int ii=0;ii<ply_files.size();ii++)
    {
        pcl::io::loadPCDFile(ply_files[ii], *cloud_in);
        //1. 滤波1
        std::cout<<"before filter size: "<<cloud_in->points.size()<<endl;
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud(cloud_in);
        vg.setLeafSize(0.01f,0.01f,0.01f);
        vg.filter(*cloud_filter);
        std::cout<<"after filter size: "<<cloud_filter->points.size()<<endl;
        //滤波2
        getPassTroughFilterData(cloud_filter, cloud_segmentation_x, "x", 0, 0.55 );
        getPassTroughFilterData(cloud_segmentation_x, cloud_segmentation_y, "y", -0.35, 0.1 );
        getPassTroughFilterData(cloud_segmentation_y, cloud_segmentation_z, "z", 0, 0.6 );
        //pcl::io::savePCDFileBinary("./pot.pcd", *cloud_segmentation_z);
        //点云上颜色
        //for(int i = 0;i<cloud_filter->points.size();i++)
        for(int i = 0;i<cloud_filter->size();i++)
        {
            //确定右侧灶面范围
            if(cloud_filter->points[i].x<0.55 && cloud_filter->points[i].x>0 && cloud_filter->points[i].y<0.1 && cloud_filter->points[i].y>-0.35)
            {
                cloud_filter->points[i].r = 255;
                cloud_filter->points[i].g = 0;
                cloud_filter->points[i].b = 0;
            }
            //确定整个灶面范围
//            if(cloud_filter->points[i].x<0.5 && cloud_filter->points[i].x>-0.5 && cloud_filter->points[i].y<0.1 && cloud_filter->points[i].y>-0.35)
//            {
//                cloud_filter->points[i].r = 255;
//                cloud_filter->points[i].g = 0;
//                cloud_filter->points[i].b = 0;
//            }
            //确定锅中心范围
//            if(cloud_filter->points[i].x<0.3 && cloud_filter->points[i].x>0.22 && cloud_filter->points[i].y<-0.15 && cloud_filter->points[i].y>-0.23)
//            {
//                cloud_filter->points[i].r = 0;
//                cloud_filter->points[i].g = 255;
//                cloud_filter->points[i].b = 0;
//            }
            //确定锅倾斜区域
//            if(cloud_filter->points[i].x<(center_x + 0.08) && cloud_filter->points[i].x>(center_x + 0.06) && cloud_filter->points[i].y<(center_y +0.08) && cloud_filter->points[i].y>(center_y -0.08))
//            {
//                cloud_filter->points[i].r = 0;
//                cloud_filter->points[i].g = 255;
//                cloud_filter->points[i].b = 0;
//            }
//            if(cloud_filter->points[i].x<(center_x - 0.06) && cloud_filter->points[i].x>(center_x - 0.08) && cloud_filter->points[i].y<(center_y +0.08) && cloud_filter->points[i].y>(center_y -0.08))
//            {
//                cloud_filter->points[i].r = 0;
//                cloud_filter->points[i].g = 255;
//                cloud_filter->points[i].b = 0;
//            }
//            if(cloud_filter->points[i].x<(center_x + 0.08) && cloud_filter->points[i].x>(center_x - 0.08) && cloud_filter->points[i].y<(center_y +0.08) && cloud_filter->points[i].y>(center_y +0.06))
//            {
//                cloud_filter->points[i].r = 0;
//                cloud_filter->points[i].g = 255;
//                cloud_filter->points[i].b = 0;
//            }
//            if(cloud_filter->points[i].x<(center_x + 0.08) && cloud_filter->points[i].x>(center_x - 0.08) && cloud_filter->points[i].y<(center_y -0.06) && cloud_filter->points[i].y>(center_y -0.08))
//            {
//                cloud_filter->points[i].r = 0;
//                cloud_filter->points[i].g = 255;
//                cloud_filter->points[i].b = 0;
//            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_segmentation_z, *cloud_segment);
        //2. 分割
        double m_clustertolerance = 0.02;//0.02能分开，0.04没分开，0。01太细了不能用，0。03能分开分出了16个，0.02分出24个（用0.02分的细一点）
        int m_minclustersize = 40;
        int m_maxclustersize = 25000;

        std::vector<pcl::PointIndices> cluster_indices;
        search_tree->setInputCloud(cloud_segment);
        // Euclidean Clustering

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;   //欧式聚类对象
        ece.setClusterTolerance (m_clustertolerance);         //设置近邻搜索的搜索半径为2cm
        ece.setMinClusterSize (m_minclustersize);             //设置一个聚类需要的最少的点数目为100
        ece.setMaxClusterSize (m_maxclustersize);             //设置一个聚类需要的最大点数目为25000
        ece.setSearchMethod (search_tree);                    //设置点云的搜索机制
        ece.setInputCloud (cloud_segment);
        ece.extract (cluster_indices);                        //从点云中提取聚类，并将点云索引保存在cluster_indices中


        pcl::copyPointCloud (*cloud_segment, *cloud_out);
        int count = 0;
        //对分割后的点云进行保存，放入向量
        for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
            it != cluster_indices.end ();
            ++it )
        {
            PointCloud::Ptr cloud_cluster (new PointCloud);
            for (std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end();++pit)
            {

                cloud_cluster->points.push_back (cloud_out->points[*pit]);

            }

            cout<<count<<endl;
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            cloud_vector.push_back(cloud_cluster);//和count计数多少没关系，分离后都添加到向量里
            stringstream view_out;
            std::string view_zero;
            if(count < 10)
            {
                view_zero = "0000";
            }else if(count >=10 && count < 100){
                view_zero = "000";
            }else{
                view_zero = "00";
            }
            view_out<<"./"<<view_zero<<count<<".pcd";
            //pcl::io::savePCDFileBinary(view_out.str(), *cloud_cluster);
            if (count < 20 )
                count++;
        }

        //3.向量中找锅点云
        PointCloud::Ptr pot_cloud(new PointCloud);
        short max_pot = 0;
        short pot_vector_size = 0;
        for(int i = 0;i<cloud_vector.size();i++)
        {
             if(cloud_vector[i]->size() > max_pot)
            {
                max_pot = cloud_vector[i]->size();
                pot_vector_size = i;
            }
        }
        for(int i = 0;i<cloud_vector.size();i++)
        {
            if(i == pot_vector_size)
            {
                pcl::copyPointCloud(*cloud_vector[i], *pot_cloud);
            }
        }

        //没有识别到锅，直接退出
        if(pot_cloud->size() < 500)
        {
            std::cout << std::endl;
            std::cout << "======================================" << std::endl;
            std::cout << "No Pot " << std::endl;
            std::cout << "======================================" << std::endl;
            std::cout << std::endl;

            viewer.showCloud(cloud_filter);
            //vector要清空
            cloud_vector.clear();
            usleep(100000000);
            continue;
        }

        pcl::io::savePCDFileBinary("./pot.pcd",*pot_cloud);

        //剔除锅把
        float pot_y_min = 0;
        for(int i = 0;i<pot_cloud->size();i++)
        {
            if(pot_cloud->points[i].y < pot_y_min)
            {
                pot_y_min = pot_cloud->points[i].y;
            }
        }
        cout<<"pot_y_min: "<<pot_y_min<<endl;

        PointCloud::Ptr pot_no_stick(new PointCloud);
        for(int i = 0;i<pot_cloud->size();i++)
        {
            if(pot_cloud->points[i].y < (pot_y_min + 0.28))
            {
                PointT p;
                p.x = pot_cloud->points[i].x;
                p.y = pot_cloud->points[i].y;
                p.z = pot_cloud->points[i].z;
                pot_no_stick->points.push_back(p);
            }
        }
        pot_no_stick->width = pot_no_stick->points.size ();
        pot_no_stick->height = 1;
        pot_no_stick->is_dense = true;
        pcl::io::savePCDFileBinary("./pot_no_stick.pcd",*pot_no_stick);

        //计算倾斜角度
        short point_one = 0;short point_two = 0;short point_three = 0;short point_four = 0;
        float height_sum_one = 0.0;float height_sum_two = 0.0;float height_sum_three = 0.0;float height_sum_four = 0.0;
        float height_avg_one = 0.0;float height_avg_two = 0.0;float height_avg_three = 0.0;float height_avg_four = 0.0;
        for(int i = 0;i<cloud_filter->size();i++)
        {
            if(cloud_filter->points[i].x<(center_x + 0.08) && cloud_filter->points[i].x>(center_x + 0.06) && cloud_filter->points[i].y<(center_y +0.08) && cloud_filter->points[i].y>(center_y -0.08))
            {
                point_one++;
                height_sum_one+=cloud_filter->points[i].z;
            }
            if(cloud_filter->points[i].x<(center_x - 0.06) && cloud_filter->points[i].x>(center_x - 0.08) && cloud_filter->points[i].y<(center_y +0.08) && cloud_filter->points[i].y>(center_y -0.08))
            {
                point_three++;
                height_sum_three+=cloud_filter->points[i].z;
            }
            if(cloud_filter->points[i].x<(center_x + 0.08) && cloud_filter->points[i].x>(center_x - 0.08) && cloud_filter->points[i].y<(center_y +0.08) && cloud_filter->points[i].y>(center_y +0.06))
            {
                point_two++;
                height_sum_two+=cloud_filter->points[i].z;
            }
            if(cloud_filter->points[i].x<(center_x + 0.08) && cloud_filter->points[i].x>(center_x - 0.08) && cloud_filter->points[i].y<(center_y -0.06) && cloud_filter->points[i].y>(center_y -0.08))
            {
                point_four++;
                height_sum_four+=cloud_filter->points[i].z;
            }
        }
        vector<float> height_avg;
        height_avg_one = height_sum_one/point_one;
        height_avg.push_back(height_avg_one);
        height_avg_two = height_sum_two/point_two;
        height_avg.push_back(height_avg_two);
        height_avg_three = height_sum_three/point_three;
        height_avg.push_back(height_avg_three);
        height_avg_four = height_sum_four/point_four;
        height_avg.push_back(height_avg_four);
        cout<<"height_avg_one: "<<height_avg_one<<" height_avg_two: "<<height_avg_two<<" height_avg_three: "<<height_avg_three<<" height_avg_four: "<<height_avg_four<<endl;
        //找最大最小值
        float height_avg_min = 1;
        float height_avg_max = 0;
        for(int i = 0;i<height_avg.size();i++)
        {
            if(height_avg[i]>height_avg_max)
            {
                height_avg_max = height_avg[i];
            }
            if(height_avg[i]<height_avg_min)
            {
                height_avg_min = height_avg[i];
            }
        }
        cout<<"max: "<<height_avg_max<<" min: "<<height_avg_min<<endl;
        float angle = 0.0;
        angle = std::atan((height_avg_max - height_avg_min)/0.18)*180/PI;
        if(angle>5 && angle < 10)
        {
            angle = angle - 5;
        }
        cout<<"angle: "<<angle<<endl;
        height_avg.clear();

        usleep(100000);
        viewer.showCloud(cloud_filter);
        //vector要清空
        cloud_vector.clear();
    }


	return (0);
}

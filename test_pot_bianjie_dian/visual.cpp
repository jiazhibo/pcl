#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
    std::string pattern_jpg;
    std::vector<cv::String> ply_files;
    pattern_jpg = "../data/*.pcd";
    cv::glob(pattern_jpg, ply_files);
    pcl::visualization::CloudViewer viewer("abc");//框的名字
    int j = 1;
    std::vector<double> ave_jihe;
    for(int ii=0;ii<ply_files.size();ii++)
    {

    string front = "../data/PointCloud_";
    string after = ".pcd";
    stringstream ss;
    ss << front << j << after;
    cout<< ss.str() <<endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::io::loadPCDFile(ss.str(), *cloud);
	pcl::io::loadPCDFile(ply_files[ii], *cloud);
	double pot_sum_z = 0.0;
	double count_pot_z = 0.0;

	for(int i = 0;i<cloud->points.size();i++)
    {
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
//        if(cloud->points[i].x<0.3 && cloud->points[i].x>0 && cloud->points[i].y<-0.03 && cloud->points[i].y>-0.06 && cloud->points[i].z<0.55)
//        {
//            cloud->points[i].r = 255;
//            cloud->points[i].g = 0;
//            cloud->points[i].b = 0;
//            pot_sum_z = pot_sum_z + cloud->points[i].z;
//            count_pot_z++;
//
//            continue;
//        }
//        if(cloud->points[i].x<0.3 && cloud->points[i].x>0 && cloud->points[i].y<-0.32 && cloud->points[i].y>-0.36 && cloud->points[i].z<0.55)
//        {
//            cloud->points[i].r = 255;
//            cloud->points[i].g = 0;
//            cloud->points[i].b = 0;
//            pot_sum_z = pot_sum_z + cloud->points[i].z;
//            count_pot_z++;
//            continue;
//        }
        //12.15.1
        //if(cloud->points[i].x<0.05 && cloud->points[i].x>0.02 && cloud->points[i].y<-0.15 && cloud->points[i].y>-0.2)
        //if(cloud->points[i].x<0.05 && cloud->points[i].x>0.04 && cloud->points[i].y<-0.15 && cloud->points[i].y>-0.2)
        //5
        //if(cloud->points[i].x<0.1 && cloud->points[i].x>0.09 && cloud->points[i].y<-0.15 && cloud->points[i].y>-0.2)
        //6
        //if(cloud->points[i].x<0.1 && cloud->points[i].x>0.09 && cloud->points[i].y<-0.15 && cloud->points[i].y>-0.2)
        //12.16.1
        if(cloud->points[i].x<0.07 && cloud->points[i].x>0.06 && cloud->points[i].y<-0.17 && cloud->points[i].y>-0.23)
        {
            cloud->points[i].r = 255;
            cloud->points[i].g = 0;
            cloud->points[i].b = 0;
            pot_sum_z = pot_sum_z + cloud->points[i].z;
            count_pot_z++;
            continue;
        }
//        if(cloud->points[i].x<0.32 && cloud->points[i].x>0.3 && cloud->points[i].y<-0.03 && cloud->points[i].y>-0.36 && cloud->points[i].z<0.55)
//        {
//            cloud->points[i].r = 255;
//            cloud->points[i].g = 0;
//            cloud->points[i].b = 0;
//            pot_sum_z = pot_sum_z + cloud->points[i].z;
//            count_pot_z++;
//            continue;
//        }
    }
    double ave_z = pot_sum_z/count_pot_z;
    ave_jihe.push_back(ave_z);
    usleep(150000);
    std::cout<<"sum_z: "<<pot_sum_z<<" count_z: "<<count_pot_z<<" ave_z: "<<ave_z<<std::endl;
    viewer.showCloud(cloud);

    for(int i = 0;i<ave_jihe.size();i++)
    {
        cout<<ave_jihe[i]<<" ";
    }
    cout<<endl;
	// 清除数据并退出
	cloud->points.clear();
	cout << "point cloud count = " << ii << endl;
    j++;
    }
	cout << "Point cloud saved." << endl;
    return 0;
}

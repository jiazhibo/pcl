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

        if(cloud->points[i].x<0.5 && cloud->points[i].x>0 && cloud->points[i].y<0.1 && cloud->points[i].y>-0.3 && cloud->points[i].z<0.5 && cloud->points[i].z>0)
        {
            cloud->points[i].r = 255;
            cloud->points[i].g = 0;
            cloud->points[i].b = 0;
            continue;
        }

    }
    usleep(150000);
    viewer.showCloud(cloud);


	// 清除数据并退出
	cloud->points.clear();
	cout << "point cloud count = " << ii << endl;
    j++;
    }
	cout << "Point cloud saved." << endl;
    return 0;
}

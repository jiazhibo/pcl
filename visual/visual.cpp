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
int j = 7;
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
usleep(200000);

viewer.showCloud(cloud);


	// 清除数据并退出
	cloud->points.clear();
	cout << "point cloud count = " << ii << endl;
j++;
    }
	cout << "Point cloud saved." << endl;
  return 0;
}

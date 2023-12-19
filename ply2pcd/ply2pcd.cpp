#include <iostream>          
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>  
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
    std::string pattern_jpg;
    std::vector<cv::String> ply_files;
    pattern_jpg = "/home/jiazhibo/pcl/ply2pcd/data/*.ply";
    cv::glob(pattern_jpg, ply_files);

    for(int ii=0;ii<ply_files.size();ii++)
    {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(ply_files[ii], *cloud);
std::string str = ply_files[ii];
int pos = str.find(".");
str = str.erase(pos,4);
	cout << "str = " << str << endl;
	string adress = "../data/pcd/";
	string after = ".pcd";
	stringstream ss;
	ss << str << after;
	cout << "str2 = " << ss.str() << endl;
	pcl::io::savePCDFile(ss.str(), *cloud);
	// 清除数据并退出
	cloud->points.clear();
	cout << "point cloud count = " << ii << endl;
    }
	cout << "Point cloud saved." << endl;
  return 0;
}

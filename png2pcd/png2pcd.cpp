#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
using namespace cv;
// 定义点云类型
//typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char** argv)
{
	// 读取./data/rgb.png和./data/depth.png，并转化为点云
	// 图像矩阵
	double mCx = 113.758644;
	double mCy = 86.060310;
	double mFx = 80.471771;
	double mFy = 80.471771;
	cv::Mat rgb, origin, depth;
	// 使用cv::imread()来读取图像
	//rgb = cv::imread("./data/rgb.png");
	//const char filename[] = "./data/depth.png";
	//“2”拿深度
	//depth = cv::imread(filename, 2);
//cout<<"depth: "<<depth<<endl;
//	origin = cv::imread(filename, 2);
//	origin.convertTo(depth, CV_32FC1);
//cout<<"depth: "<<depth<<endl;



    std::string pattern_jpg;
    std::vector<cv::String> image_files;
    pattern_jpg = "./data/*.png";
    cv::glob(pattern_jpg, image_files);
    for(int ii=0;ii<image_files.size();ii++)
    {

	depth = cv::imread(image_files[ii], 2);
	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
	PointCloud::Ptr cloud(new PointCloud);
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			ushort d = depth.ptr<ushort>(m)[n];
			if (d == 0)
				continue;
			PointT p;
			p.z = double(d)/1000;			
			p.x = (n - mCx)*p.z/mFx;
			p.y = (m - mCy)*p.z/mFy;
			
			

			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色。
			//p.b = rgb.ptr<uchar>(m)[n * 3];;
			//p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			//p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            //如果不需要将点云上色，可改成单一色。
			p.b = 255;;
			p.g = 255;
			p.r = 255;

			// 把p加入到点云中
			cloud->points.push_back(p);
		}
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;

	string adress = "./data/pcd/";
	string after = ".pcd";
	stringstream ss;
	ss << adress << ii << after;


	try {
		//保存点云图
		pcl::io::savePCDFile(ss.str(), *cloud);
	}
	catch (pcl::IOException &e) {
		cout << e.what() << endl;
	}

	// 清除数据并退出
	cloud->points.clear();
}
	cout << "Point cloud saved." << endl;
	return 0;
}


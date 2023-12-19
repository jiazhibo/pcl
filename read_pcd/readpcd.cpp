#include <iostream> //标准c++库输入输出相关头文件
#include <pcl/io/pcd_io.h> // pcd读写相关头文件
#include <pcl/point_types.h> // pcl中支持的点类型头文件
// 定义点云格式，具体见下章
typedef pcl::PointXYZ PointT;
using namespace std;
int main(int argc, char** argv)
{
	// 定义点云
	pcl::PointCloud<PointT>::Ptr cloud_ptr1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_ptr2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT> cloud;

	// 读取点云，失败返回-1
	if (pcl::io::loadPCDFile<PointT>("00001.pcd", *cloud_ptr1) == -1)
	{
		PCL_ERROR("couldn't read file\n");
		return (-1);
	}
	// 输出点云大小 cloud->width * cloud->height
	std::cout << "点云大小：" << cloud_ptr1->size() << std::endl;
	std::cout << "点云大小：" << cloud_ptr2->size() << std::endl;
	std::cout << "点云大小：" << cloud.size() << std::endl; 
	std::cout << "x：" << cloud_ptr1->points[0].x << "y：" << cloud_ptr1->points[0].y << "z：" << cloud_ptr1->points[0].z << std::endl; 

	//cloud_ptr2 = cloud_ptr1;
	//cloud = *cloud_ptr1;
	pcl::copyPointCloud(*cloud_ptr1, *cloud_ptr2);
	pcl::copyPointCloud(*cloud_ptr1, cloud);

	std::cout << "点云大小：" << cloud_ptr1->size() << std::endl;
	std::cout << "点云大小：" << cloud_ptr2->size() << std::endl;
	std::cout << "点云大小：" << cloud.size() << std::endl; 
	std::cout << "x：" << cloud_ptr1->points[0].x << "y：" << cloud_ptr1->points[0].y << "z：" << cloud_ptr1->points[0].z << std::endl;
	std::cout << "x：" << cloud_ptr2->points[0].x << "y：" << cloud_ptr2->points[0].y << "z：" << cloud_ptr2->points[0].z << std::endl;
	std::cout << "x：" << cloud.points[0].x << "y：" << cloud.points[0].y << "z：" << cloud.points[0].z << std::endl;	

	cloud_ptr1->points[0].x = 0.5;
	cloud_ptr1->points[0].y = 0.5;
	cloud_ptr1->points[0].z = 0.5;
	std::cout << "result" << std::endl;
	std::cout << "x：" << cloud_ptr1->points[0].x << "y：" << cloud_ptr1->points[0].y << "z：" << cloud_ptr1->points[0].z << std::endl;
	std::cout << "x：" << cloud_ptr2->points[0].x << "y：" << cloud_ptr2->points[0].y << "z：" << cloud_ptr2->points[0].z << std::endl;
	std::cout << "x：" << cloud.points[0].x << "y：" << cloud.points[0].y << "z：" << cloud.points[0].z << std::endl;














	return (0);
}

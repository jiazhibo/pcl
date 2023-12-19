#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT; // 也可以pcl::Normal,但无法用PCLVisualizer显示。
int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	
#if 0
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{	// 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
		// 也可用其他方法进行连接，如：pcl::concatenateFields
		normals->points[i].x = cloud->points[i].x;
		normals->points[i].y = cloud->points[i].y;
		normals->points[i].z = cloud->points[i].z;
	}
#endif
	// 显示
double a = 2.5;
int b = 2;
double c = a/b;
cout<<c<<endl;
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");

	viewer.spin();
	system("pause");
	return 0;
}

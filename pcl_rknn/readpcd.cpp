#include <math.h>
#include <opencv2/opencv.hpp> // For visualizing image
#include <pcl/point_cloud.h>  // For PCL Point Cloud
#include <pcl/io/pcd_io.h>    // For Reading the Cloud
#include <pcl/point_types.h>  // For PCL different cloud types

#include <vector>

void GetProjection(const pcl::PointXYZI& point,
                                        const double& fov_rad,
                                        const double& fov_down_rad,
                                        int* pixel_v, int* pixel_u,
                                        double* range) {
    // range of Point from Lidar
    *range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
std::cout<<" range: "<<*range<<" ";
    //  Getting the angle of all the Points
    //auto yaw = atan2(point.y, point.x);
    //auto pitch = asin(point.z / *range);
    auto yaw = -atan2(point.x, point.z);
    auto pitch = asin(point.y / *range);

    // Get projections in image coords and normalizing
    //double v = 0.5 * (yaw / M_PI + 1.0);
    double v = 1.5 * (yaw / M_PI + 1.0/3);
    double u = 1.0 - (pitch + std::abs(fov_down_rad)) / fov_rad;
    // Scaling as per the lidar config given
//std::cout<<" v0: "<<v<<" ";
    v *= 64;
    u *= 43;
    // round and clamp for use as index
    //std::cout<<" v0: "<<v<<" ";
    v = floor(v);
    //std::cout<<" v1: "<<v<<" ";
    v = std::min(63.0, v);
    v = std::max(0.0, v);
    //std::cout<<" v2: "<<v<<" ";
    *pixel_v = int(v);
    //std::cout<<" u0: "<<u<<" ";
    u = floor(u);
    //std::cout<<" u1: "<<u<<" ";
    u = std::min(42.0, u);
    u = std::max(0.0, u);
    //std::cout<<" u2: "<<u<<" ";
    *pixel_u = int(u);
}
int main() {
int fov_up = 45;
int fov_down = -45;
int num_lasers = 43;
int img_length = 64;
std::vector<std::vector<std::vector<double>>> spherical_img_;
    spherical_img_.assign(num_lasers,
                          std::vector<std::vector<double>>(
                              img_length, std::vector<double>(3, -1.0)));

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("00001.pcd", *cloud_) == -1) {
        std::cout << "Couldn't read the cloud file at: " << "\n";
    }

    double fov_up_rad = (fov_up / 180) * M_PI;
    double fov_down_rad = (fov_down / 180) * M_PI;
    // Getting total Field of View
    double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
    if (cloud_->size() == 0) {
        std::cerr << "Empty Point Cloud_" << std::endl;
        return -1;
    }

    for (auto point : *cloud_) {
        // Getting Pixel from Point
        int pixel_v = 0;
        int pixel_u = 0;
        double range = 0.0;
        GetProjection(point, fov_rad, fov_down_rad, &pixel_v, &pixel_u, &range);
        //spherical_img_.at(pixel_u).at(pixel_v) = std::vector<double>{
        //    point.x, point.y, point.z, range, point.intensity};
	//printf("pixel_u: %d,pixel_v: %d\n",pixel_u,pixel_v);
        //spherical_img_.at(pixel_u).at(63 - pixel_v) = std::vector<double>{point.x, point.y, point.z};
	//spherical_img_.at(pixel_u).at(pixel_v) = std::vector<double>{point.x, point.y, point.z};
//std::cout<<" pixel_u: "<<pixel_u<<"pixel_v: "<<pixel_v<<" ";
	//spherical_img_.at(0).at(pixel_u).at(63 - pixel_v) = std::vector<double>{point.x, point.y, point.z};

    }
}


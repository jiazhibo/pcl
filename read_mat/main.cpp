#include "opencv2/opencv.hpp"
#include "iostream"
 
using namespace std;
using namespace cv;
 
 
int main()
{

	cv::Mat mat = imread("../01.jpg");
	std::cout << "channel = " << mat.channels() << endl;
	std::cout << "row = " << mat.rows << endl;
	std::cout << "col = " << mat.cols << endl;
	cout <<mat << endl;
	cout << "test" <<endl;
	return 0;
}

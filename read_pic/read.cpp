
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
    std::string pattern_jpg;
    std::vector<cv::String> image_files;
    pattern_jpg = "./data/*.png";
    cv::glob(pattern_jpg, image_files);
    for(int ii=0;ii<image_files.size();ii++)
    {
        cout << image_files[ii] << endl;
        namedWindow("dd");
   		Mat dd=imread(image_files[ii]);
    	imshow("dd",dd);
    	
    }   
	waitKey();

	return 0;
}


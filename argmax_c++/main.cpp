#include "opencv2/opencv.hpp"
#include "iostream"
 
using namespace std;
using namespace cv;

#include <algorithm>

#if 0
template<class ForwardIterator>
inline size_t argmin(ForwardIterator first, ForwardIterator last)
{
    return std::distance(first, std::min_element(first, last));
}
template<class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last)
{
    return std::distance(first, std::max_element(first, last));
}

int main() {
    array<int, 7> numbers{2, 4, 8, 0, 6, -1, 3};
    size_t minIndex =  argmin(numbers.begin(), numbers.end());
    cout << minIndex << '\n';
    vector<float> prices = {7.55190849e-05, 1.08051300e-03, 2.89916992e-03, 3.20068359e-01, 0};
    size_t maxIndex = argmax(prices.begin(), prices.end());
    cout << maxIndex << '\n';
    return 0;
}
#endif

int main() {

		std::vector<std::vector<std::vector<double>>> mOutputsVector; 

    std::vector<std::vector<int>> mProjArgmax;
    mOutputsVector.assign(8,std::vector<std::vector<double>>(1, std::vector<double>(1, 0.0)));

    mProjArgmax.assign(1, std::vector<int>(1, 0));

int w = 5;
vector<int> a;
a.push_back(w);
cout<< a[0] << endl;
#if 0
0 
jjj
0

jjj
0.00108337 0.000480175 0.000270367 0.000176191 0.000102878 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 6.4671e-05 


jjj
0.262695 0.762207 0.515625 0.851562 0.89209 0.98584 0.940918 0.893066 0.366211 0.499512 0.609375 0.955566 

jjj
0.73584 0.236206 0.482178 0.147095 0.102173 0.0116348 0.0478516 0.0556641 0.0707397 0.133179 0.270752 0.0334778 
jjj
0.00168991 0.000163674 0 0.000526428 0.000941753 0.00181675 0.000917912 0.00514984 0.00210571 0.0107422 0.0506287 0.562988 0.367188 
jjj
0.000314951 0 0 0 0 6.73532e-05 0.00013268 0.000593185 0.000451088 0.000373363 0.000282526 0.000111103 0.000103116 0.000161648 8.95262e-05 

jjj
0.00037384 0 0 0 0 0 0 0 0 0 0.000289917 0.000221968 7.82013e-05 0 0 0 0 0 0 0 0 0 0 0 0 0 6.70552e-05 0.000644207 0.000226378 0.00286102 


[[[0.00000000e+00


 [[0.00000000e+00
   

 [[1.05857849e-03
   

 [[2.55126953e-01
   

 [[7.43164062e-01
   

 [[5.18798828e-04
   

 [[0.00000000e+00
   

 [[0.00000000e+00
   

    for (int i = 0; i < 3; i++)
    {
		for (int j=0; j<4; j++)
		{
			for(int k=0; k<5; k++)
			{
				mOutputsVector.at(i).at(j).at(k) = k-j-i;
			}
		}
    }
#endif
mOutputsVector.at(0).at(0).at(0) = 7.55190849e-05;
mOutputsVector.at(1).at(0).at(0) = 1.08051300e-03;
mOutputsVector.at(2).at(0).at(0) = 2.89916992e-03;
mOutputsVector.at(3).at(0).at(0) = 3.20068359e-01;
mOutputsVector.at(4).at(0).at(0) = 6.73339844e-01;
mOutputsVector.at(5).at(0).at(0) = 1.68228149e-03;
mOutputsVector.at(6).at(0).at(0) = 3.11374664e-04;
mOutputsVector.at(7).at(0).at(0) = 3.73601913e-04;



    for (int i = 0; i < 8; i++)
    {
		for (int j=0; j<1; j++)
		{
			for(int k=0; k<1; k++)
			{
				cout<< mOutputsVector.at(i).at(j).at(k) << " ";
			}
			cout<< endl;
		}
	cout<< endl;
    }
	cout<< "********************************************************************8" << endl;
    int maxIndex;
    vector<double> compare;
    int flag = 0;

	for (int j=0; j<1; j++)
	{
		for(int k=0; k<1; k++)
		{
		    for (int i = 0; i < 8; i++)
		    {
			compare.push_back(mOutputsVector.at(i).at(j).at(k));
			cout<< "compare" <<compare[i] <<" ";
    		    }
			cout<< endl;
		    maxIndex = std::distance(compare.begin(), std::max_element(compare.begin(), compare.end()));
			cout <<compare.size() <<endl;
	
		for(int i = 1; i<compare.size();i++)
		{
			if(compare[i] == compare[i-1])
			{
				flag++;
			}	
		}
		cout<<"flag: "<<flag<<endl;

		if(flag == 7)
		{	
			maxIndex = 7;
			flag = 0;
		}

		cout<< "maxIndex" << maxIndex <<endl;
		    mProjArgmax.at(j).at(k) = maxIndex;
			compare.clear();
		}
	}
	for (int j=0; j<1; j++)
	{
		for(int k=0; k<1; k++)
		{
			cout<< mProjArgmax.at(j).at(k) << " ";
		}
	cout<< endl;
	}

    return 0;
}
























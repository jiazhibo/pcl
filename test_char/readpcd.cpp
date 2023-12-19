#include <iostream>
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <vector>
using namespace std;
typedef struct pc_pack{
  float x;
  float y;
  float z;
  float c;
}pc_pkt_t;
int main(int argc, char** argv)
{
char tcp_data[2*sizeof(pc_pkt_t)];

int a = 5;
std::cout<<sizeof(tcp_data)<<std::endl;

std::cout<<sizeof(a)<<std::endl;

vector<double> j = {1,2};
cout<<j.size()<<endl;
cout<<j.at(0)<<endl;
cout<<"argc "<<argc<<endl;
cout<<"argv[1] : "<<argv[1]<<endl;
	return 0;
}

#include <iostream>
#include <string>
#include <sstream>
using namespace std;
int main()
{
    int a = 2;
    string b = "abc";
string c = "de";
    stringstream ss;
    ss << b << a << c;
    cout << ss.str() << endl;
    return 0;
}

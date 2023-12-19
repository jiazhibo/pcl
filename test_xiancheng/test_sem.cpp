#include <iostream>
#include <unistd.h>
//#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <queue>
//#include <semaphore.h>
#include <mutex>

using namespace std;
std::queue<int> test_queue;
std::mutex mGetDataMutex;
boost::mutex mGetDataMutex_2;
//boost::shared_ptr<std::queue<cv::Mat>> test_queue = NULL;
//sem_t tempty;
//sem_t occupied;
//sem_t p_lock;
//sem_t c_lock;
void getData()
{
    while(true)
    {
//        sem_wait(&occupied);
//        sem_wait(&c_lock);
boost::unique_lock < std::mutex > lock ( mGetDataMutex );
cout<<"777777777 "<<endl;
        int value = test_queue.front();
        int a = test_queue.front();
        int b = test_queue.front();
cout<<"888888888 "<<endl;
usleep(2000);
        int c = test_queue.front();
        int d = test_queue.front();
        int e = test_queue.front();
        int f = test_queue.front();
cout<<"999999999 "<<endl;
lock.unlock();
//        sem_post(&c_lock);
//        sem_post(&tempty);
        cout<<"thread front: "<<value<<endl;
        test_queue.pop();
        usleep(2000);
    }
}
void delete_queue()
{
    if(test_queue.size()>3)
    {
        test_queue.pop();
    }
}
int main(int argc, char** argv)
{
    boost::thread * IRLoopThread = new boost::thread(boost::bind(&getData));
    short count = 0;
//    sem_init(&tempty, 0, 3);
//    sem_init(&occupied, 0, 0);
//    sem_init(&p_lock, 0, 1);
//    sem_init(&c_lock, 0, 1);
    while(true)
    {

        test_queue.push(count);

//        sem_wait(&tempty);
//        sem_wait(&p_lock);
boost::unique_lock < std::mutex > lock2 ( mGetDataMutex );
cout<<"111111111 "<<endl;
        delete_queue();
cout<<"222222222 "<<endl;
usleep(2000);
        int w = test_queue.front();
cout<<"333333333 "<<endl;
//        sem_post(&p_lock);
//        sem_post(&occupied);
lock2.unlock();
        cout<<"main front: "<<w<<endl;
        count++;
        usleep(1000);
    }
}

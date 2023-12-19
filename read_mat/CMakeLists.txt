
cmake_minimum_required (VERSION 2.6)
project (read_mat)
 
set (TEST_VERSION 0.1)
 
find_package(OpenCV REQUIRED)
 
#添加OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})
 
#显示OpenCV_INCLUDE_DIRS的值
message(${OpenCV_INCLUDE_DIRS})
 
# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( main main.cpp )
 
# 将库文件链接到可执行程序上
target_link_libraries( main  ${OpenCV_LIBS})


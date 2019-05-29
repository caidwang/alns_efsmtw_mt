#CMakeList.txt
# 最低cmake版本号
cmake_minimum_required(VERSION 3.2)
# 项目名
project(alns_efsmtw_mt)
# cpp版本
set(CMAKE_CXX_STANDARD 11)
# 添加动态链接库搜索目录
link_directories(lib)
# 设置静态链接库搜索目录
include_directories(alns_framework/src)
include_directories(include)
# 将当前目录的所有cpp文件 赋给变量dir_srcs
aux_source_directory(src DIR_SRCS)
# 指定生成目标
add_executable(${PROJECT_NAME} ${DIR_SRCS})
# 将生成目标与必要的动态库链接
target_link_libraries(${PROJECT_NAME} ALNS-framework)
target_link_libraries(${PROJECT_NAME} pthread)
#CMakeList.txt
# 最低cmake版本号
cmake_minimum_required(VERSION 3.2)
# 项目名
project(alns_efsmtw_mt)
# cpp版本
set(CMAKE_CXX_STANDARD 11)
# 添加动态链接库搜索目录
#link_directories(lib)
# 设置静态链接库搜索目录
set(LIBDIR /home/wsc/code/optimism_projects/ALNS_framework/ALNS/alns-framework/trunk/src)

include_directories(${LIBDIR})



include_directories(include)
# 将当前目录的所有cpp文件 赋给变量dir_srcs
aux_source_directory(src DIR_SRCS)

aux_source_directory(${LIBDIR}/acceptanceModule SRC1)
aux_source_directory(${LIBDIR}/alns SRC2)
aux_source_directory(${LIBDIR}/lib/tinyxml SRC3)
aux_source_directory(${LIBDIR}/localsearch SRC4)
aux_source_directory(${LIBDIR}/statistics SRC5)
aux_source_directory(${LIBDIR}/tuning SRC6)

# 指定生成目标
add_executable(${PROJECT_NAME} ${DIR_SRCS} ${SRC1} ${SRC2} ${SRC3} ${SRC4} ${SRC5} ${SRC6})

# 将生成目标与必要的动态库链接
#target_link_libraries(${PROJECT_NAME} ALNS-framework)
target_link_libraries(${PROJECT_NAME} pthread)
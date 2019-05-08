#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include "logistics.hpp"
#include "VRP_Solution.h"

#ifndef UTIL_H
#define UTIL_H


/**
 * data preprocess
 * 从dist_mat_file读文件, 共有nnode个节点, 结果写入dist_mat和time_mat
 * 其中dist_mat[i][j]为点i到点j的距离, time_mat[i][j]为点i到点j的时间
 */
void read_dist_time_mat(const std::string &dist_mat_file,
                        std::vector<std::vector<int>> &dist_mat,
                        std::vector<std::vector<int>> &time_mat, int nnode);

/**
 * 从文件读节点信息
 * for all Nodes: id, type={1,2,3}, lng, lat, time is recorded in minutes.
 * angle is the angle between line that links node to depot and north.
 * for depot and charge station: demand_weight = demand_capacity = 0
 * for charge station: early_time = 0, last_time = 24*60
 */
void read_nodes(const std::string &file_path, std::vector<Node> &node_list, bool has_header=true);



//float cal_angle(float, float);

// data visualise
// function for inspect the dist_mat and time_mat
// 打印最后5行
void print_mat(std::vector<std::vector<int> > &mat);

void print_node_list(std::vector<Node> &, int, int);

// 打印结果 形式为第一行 总路径数n, 之后n行id->id->id 车辆类型
void print_solution(VRP_Solution &solution, std::ostream &out, bool debug);
bool lastest_modified_file(const char *dir_name,std::string & name);
void write_answer(ISolution *sol);
// 打印结果 形式为第一行 总路径数n,
// 之后n行 车辆类型 车辆id 路径载重率 路径容量率 id->id->id
void analyse_result(const std::vector<Route> &route_list);

//int cal_total_cost(const std::vector<Route> &route_list);

// unit tests
void test_insert_and_drop();
void test_check_time_violation();
void test_check_distance_violation();
void test_check_capacity_violation();
void test_find_charger();
#endif
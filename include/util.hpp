#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <string>
#include "logistics.hpp"

#ifndef UTIL_H
#define UTIL_H

// data preprocess
void read_dist_time_mat(const std::string &);
void read_nodes(std::vector<Node> &, const std::string &);

float cal_angle(float, float);

// data visualise
void print_mat(std::vector<std::vector<int> > &mat);
void print_result(const std::vector<Route> &rout_list);
void analyse_result(const std::vector<Route> &route_list);
int cal_total_cost(const std::vector<Route> &route_list);
void print_node_list(std::vector<Node> &, int, int);

// unit tests
void test_insert_and_drop();
void test_check_time_violation();
void test_check_distance_violation();
void test_check_capacity_violation();
void test_find_charger();
#endif
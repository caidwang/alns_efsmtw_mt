//
// Created by wsc on 19-5-3.
//

#ifndef ALNS_EFSMTW_MT_RELATEDNESS_H
#define ALNS_EFSMTW_MT_RELATEDNESS_H
#include<vector>
#include<queue>
#include "logistics.hpp"
// get_relatedness parameters
# define DISTANCE_WEIGHT 1
# define WAITING_TIME_WEIGHT 0.2
# define TIME_WINDOW_WEIGHT 1



class Relatedness {
public:
    Relatedness(const std::vector<Node> &node_list, const std::vector<std::vector<int>> &dist_mat,
                const std::vector<std::vector<int>> &time_mat, int n_customers);
    int get_relatedness(int node1, int node2) const;
    int related_RCL(int base_node, int num_of_candidate) const;
private:
    const double distance_weight = DISTANCE_WEIGHT, waiting_time_weight = WAITING_TIME_WEIGHT, time_window_weight=TIME_WINDOW_WEIGHT;
    const std::vector<std::vector<int>> &dist_mat;
    const std::vector<std::vector<int>> &time_mat;
    const std::vector<Node> &node_list;
    const int n_customers;
    std::vector<std::vector<int>> relatedness_table;

};

#endif //ALNS_EFSMTW_MT_RELATEDNESS_H

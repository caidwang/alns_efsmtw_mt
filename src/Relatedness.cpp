//
// Created by wsc on 19-5-3.
//

#include "Relatedness.h"
using namespace std;

class RelatednessLess {
public:
    RelatednessLess(int base, const std::vector<std::vector<int>> &table) : base (base), table(table) {}
    bool operator()(int i, int j) {
        return table[base][i] > table[base][j];
    }
private:
    const std::vector<std::vector<int>> &table;
    int base;
};

Relatedness::Relatedness(const std::vector<Node> &node_list, const std::vector<std::vector<int>> &dist_mat,
                         const std::vector<std::vector<int>> &time_mat, int n_customers) :
        dist_mat(dist_mat), time_mat(time_mat), node_list(node_list), n_customers(n_customers) {
    int N = node_list.size();
    for (int i = 0; i < N; ++i) {
        relatedness_table.emplace_back(N);
        for (int j = 0; j < N; ++j) {
            if (i == j)
                relatedness_table[i][j] = INT32_MAX;
            else if (j < i)
                relatedness_table[i][j] = relatedness_table[j][i];
            else
                relatedness_table[i][j] = static_cast<int>(distance_weight*dist_mat[i][j]) +
                                          static_cast<int>(max(0, node_list[j].early_time - (node_list[i].last_time + OPERATION_TIME + time_mat[i][j])) * waiting_time_weight) +
                                          static_cast<int>(max(0, node_list[i].early_time + OPERATION_TIME + time_mat[i][j] - node_list[j].last_time) * time_window_weight);
        }
    }
}
int Relatedness::get_relatedness(int node1, int node2) const {
    return relatedness_table[node1][node2];
}

int Relatedness::related_RCL(int base_node, int num_of_candidate) const {
    RelatednessLess rLess(base_node, relatedness_table);
    vector<int> RCL;
    for (int i = 1; i <= n_customers; ++i) {
        RCL.push_back(i);
    }
    sort(RCL.begin(), RCL.end(), rLess);
    return RCL[rand() % num_of_candidate];
}

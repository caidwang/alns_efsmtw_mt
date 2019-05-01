//
// Created by wsc on 19-4-27.
//

#include "VRP_Solution.h"
#include <memory>
using namespace std;

VRP_Solution::VRP_Solution(const vector<Node> *node_list,
        const vector<vector<int>> *dist_mat,
        const vector<vector<int>> *time_mat,
        int n_cust, int n_node)  : node_list(node_list), dist_mat(dist_mat), time_mat(time_mat), n_customers(n_cust), n_nodes(n_node), cost(0) {
    for (int i = 0; i <= n_cust; ++i) {
        nonInserted.push_back(i);
    }
}

int VRP_Solution::distance(ISolution &) {
    return 0;
}

long long VRP_Solution::getHash() {
    long long ret = 0;
    for (const auto &route : routes) {
        for (auto node: route.route_seq) {
            ret = ret * 1101 + node;
        }
    }
    return ret;
}

ISolution *VRP_Solution::getCopy() {
    auto new_S = new VRP_Solution(node_list, dist_mat, time_mat, n_customers, n_nodes);
    new_S->cost = cost;
    new_S->routes = routes;
    new_S->nonInserted = nonInserted;
    return dynamic_cast<ISolution*>(new_S);
}

bool VRP_Solution::operator<(ISolution &s) {
    return getPenalizedObjectiveValue() < s.getPenalizedObjectiveValue();
}

const vector<Route> &VRP_Solution::getRoutes() const {
    return routes;
}

const vector<int> &VRP_Solution::getNonInserted() const {
    return nonInserted;
}

int VRP_Solution::getNNodes() const {
    return n_nodes;
}

int VRP_Solution::getNCustomers() const {
    return n_customers;
}



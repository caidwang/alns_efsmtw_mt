//
// Created by wsc on 19-4-27.
//

#include "VRP_Solution.h"
#include <fstream>
#include <sstream>
#include <memory>
#include <set>
using namespace std;

VRP_Solution::VRP_Solution(const vector<Node> *node_list,
        const vector<vector<int>> *dist_mat,
        const vector<vector<int>> *time_mat,
        int n_cust, int n_node)  : node_list(node_list), dist_mat(dist_mat), time_mat(time_mat), n_customers(n_cust), n_nodes(n_node) {
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
//    new_S->penalizedCost = penalizedCost;
    new_S->routes = routes;
    new_S->nonInserted = nonInserted;
    return dynamic_cast<ISolution*>(new_S);
}

bool VRP_Solution::operator<(ISolution &s) {
    return getPenalizedObjectiveValue() < s.getPenalizedObjectiveValue();
}

std::vector<Route> &VRP_Solution::getRoutes() {
    return routes;
}

vector<int> &VRP_Solution::getNonInserted(){
    return nonInserted;
}

int VRP_Solution::getNNodes() const {
    return n_nodes;
}

int VRP_Solution::getNCustomers() const {
    return n_customers;
}

// 从文件读solution到Solution对象, 用于断点继续和初始解载入
void read_vrp_solution_from_file(const std::string &file_path, VRP_Solution &solution) {
    fstream in(file_path);
    if (in) {
        int v_id = 0, v_type = 1, start_time = 480, r_id;
        set<int> not_insert;
        for (int i  = 1; i <= solution.n_customers; ++i)
            not_insert.insert(i);
        vector<int> id_sequence;
        string line, block;
        auto &routes = solution.routes;
        // 解的格式
        // vehicle_id, vehicle_type, id->id->id, start_time, back_time, total_dist
        while (getline(in, line)){
            istringstream is(line);
            for (int i = 0; i < 4; ++i) {
                getline(is, block, ',');
                if (i == 0) {
                    v_id = stoi(block);
                }
                else if (i == 1) {
                    v_type = stoi(block);
                }
                else if (i == 2) {
                    istringstream seq(block);
                    while(seq) {
                        seq >> r_id;
                        seq.ignore(2);
                        id_sequence.push_back(r_id);
                    }
                }
                else if (i == 3) {
                    start_time = stoi(block);
                }
            }
            Route cur_route(v_id, v_type, start_time);
            for (auto it = id_sequence.begin() + 1; it != id_sequence.end() - 1; ++it) {
                cur_route.lazy_insert(*it, cur_route.route_seq.size() - 1);
                not_insert.erase(*it);
                // todo 总是检查是否需要更新route的属性信息 当前通过insert方法 全部插入后一次更新
            }
            cur_route.update(0, cur_route.size() - 1);
            routes.push_back(cur_route);
            id_sequence.clear();
        }
        solution.nonInserted.clear();
        for (int it : not_insert) {
            solution.nonInserted.push_back(it);
        }
        // todo solution 中的其他属性 还要更新
    }
    else __throw_ios_failure("file not exists.");
}

bool VRP_Solution::isFeasible() {
    if (!nonInserted.empty()) return false;
    for(auto &route : routes) {
        if (!route.isFeasible())
            return false;
    }
    return true;
}

double VRP_Solution::getPenalizedObjectiveValue() {
    double object = 0;
    for (auto &route : routes) {
        object += route.get_penalized_cost();
    }
    return object;
}

double VRP_Solution::getObjectiveValue() {
    double object = 0;
    for (auto &route : routes) {
        object += route.get_object_cost();
    }
    return object;
}

void VRP_Solution::remove(std::vector<int> &remove_list) {
    // todo 加入节点的位置信息 进行重构
    assert(remove_list_is_unique(remove_list));
    set<int> waiting_for_remove;
    for (auto node: remove_list) {
        nonInserted.push_back(node);
        waiting_for_remove.insert(node);
    }
    for (auto &route : routes) {
        bool changed = false;
        for (int i = 1; i < route.size() - 1; ++i) {
//            auto set_it = waiting_for_remove.find(*it);
            int node_id = route.get_node_by_position(i);
            if (waiting_for_remove.find(node_id) != waiting_for_remove.end()) {
                changed = true;
                waiting_for_remove.erase(node_id);
                route.lazy_remove(i);
                --i;
            }
        }
        if (changed) route.update(0, route.size());
    }
    assert(waiting_for_remove.empty());
}




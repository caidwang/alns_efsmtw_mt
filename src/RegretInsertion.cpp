//
// Created by wsc on 19-5-2.
//

#include "RegretInsertion.h"
#include "VRP_Solution.h"
#include "logistics.hpp"
using namespace std;

void RegretInsertion::repairSolution(ISolution &sol) {
    auto &newS = dynamic_cast<VRP_Solution &>(sol);
    int num_of_routes = newS.getRoutes().size();
    int num_of_nodes = newS.getNonInserted().size();
    auto &routes = newS.getRoutes();
    auto &not_insert = newS.getNonInserted();

    vector<vector<InsertInfo>> pos_cost(num_of_nodes, vector<InsertInfo>(num_of_routes));  //每个客户点在不同路径的最优插入位置及对应的cost
    for (auto &route : newS.getRoutes()) {
        // todo update routes to newest states.
        route.update();
    }
    for(auto &i : pos_cost)
        for (auto &j: i)
            j.cost = INF;

    vector<bool> visited(num_of_nodes, false);
    // 计算了所有节点在所有插入位置的cost 保存每个节点在每个路径的最小插入cost
    for (int id_in_nonInsert = 1; id_in_nonInsert < num_of_nodes; ++id_in_nonInsert) {
        for (int route_id = 0; route_id < num_of_routes; ++route_id) {
            for (int cur_pos = 1; cur_pos < routes[route_id].size(); ++cur_pos) {
                auto info = evaluate_insert_with_rs(routes[route_id], route_id, cur_pos, not_insert[id_in_nonInsert]);
                //todo 这里可以用定长的priority_queue重构
                if (info.cost < pos_cost[id_in_nonInsert][route_id].cost) {
                    pos_cost[id_in_nonInsert][route_id] = info;
                }
            }
        }
    }
    while (true) {
        vector<double> regret_value(num_of_nodes);
        // 计算所有点的regret值 计算公式 sum_{i=1}^k{c_i - c_0}
        for(int i = 0; i < num_of_nodes; ++i) {
            regret_value[i] = 0;
            if (!visited[i]) {
                sort(pos_cost[i].begin(), pos_cost[i].end(), RCLLess());
                for (int j = 1; j < k; ++j) {
                    regret_value[i] += pos_cost[i][j].cost - pos_cost[i][0].cost;
                }
            }
        }
        // 找到regret value最大的未插入的点
        int hurriest_node = -1;
        double hurriest_cost = 0;
        for (int i = 0; i < num_of_nodes; ++i) {
            if (regret_value[i] < hurriest_cost) {
                hurriest_cost = regret_value[i];
                hurriest_node = i;
            }
        }
        if (hurriest_cost == 0) break;
        // 插入最急的点 注意hurriest的id是not_insert的id
        auto &info_for_insert = pos_cost[hurriest_node][0];
        do_insert_from_info(routes[info_for_insert.cur_route], info_for_insert, not_insert[hurriest_node]);
        visited[hurriest_node] = true;

        // 剩余点的pos_cost

        // 重构成新的函数
        for (int id_in_nonInsert = 1; id_in_nonInsert < num_of_nodes; ++id_in_nonInsert) {
            if (visited[id_in_nonInsert]) continue;
            for (int route_id = 0; route_id < num_of_routes; ++route_id) {
                for (int cur_pos = 1; cur_pos < routes[route_id].size(); ++cur_pos) {
                    auto info = evaluate_insert_with_rs(routes[route_id], route_id, cur_pos, not_insert[id_in_nonInsert]);
                    //todo 这里可以用定长的priority_queue重构
                    if (info.cost < pos_cost[id_in_nonInsert][route_id].cost) {
                        pos_cost[id_in_nonInsert][route_id] = info;
                    }
                }
            }
        }
    }
    vector<int> nonInsertNew;
    for (int i = 0; i < num_of_nodes; ++i) {
        if (visited[i] == false) nonInsertNew.push_back(not_insert[i]);
    }
    not_insert = nonInsertNew;
}

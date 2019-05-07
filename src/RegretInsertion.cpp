//
// Created by wsc on 19-5-2.
//

#include "RegretInsertion.h"
#include "VRP_Solution.h"

using namespace std;

void RegretInsertion::repairSolution(ISolution &sol) {
#ifdef CAL_TIME
    time_t start = clock();
#endif
    auto &newS = dynamic_cast<VRP_Solution &>(sol);
    int num_of_routes = newS.getRoutes().size();
    int num_of_nodes = newS.getNonInserted().size();
    auto &routes = newS.getRoutes();
    auto &not_insert = newS.getNonInserted();

    vector<vector<InsertInfo>> pos_cost(num_of_nodes, vector<InsertInfo>(num_of_routes));  //每个客户点在不同路径的最优插入位置及对应的cost

    for(auto &i : pos_cost)
        for (auto &j: i)
            j.cost = INF;

//    for (auto &route : newS.getRoutes()) {
        // todo update routes to newest states. 如果能够保证route的每次插入删除的数据更新完备性 这个是不需要的
//        route.update(0, route.size());
//    }

    visited.resize(num_of_nodes);
    fill(visited.begin(), visited.end(), false);

    // 计算了所有节点在所有插入位置的cost 保存每个节点在每个路径的最小插入cost
    update_position_cost(not_insert, routes, pos_cost);
    while (true) {
        vector<double> regret_value(num_of_nodes);
        // 计算所有点的regret值 计算公式 sum_{i=1}^k{c_i - c_0}
        calculate_regret_value(pos_cost, regret_value);
        // 找到regret value最大的未插入的点
        int hurriest_node = -1;
        double hurriest_cost = -1;
        for (int i = 0; i < num_of_nodes; ++i) {
            if (!visited[i] && regret_value[i] > hurriest_cost) {
                hurriest_cost = regret_value[i];
                hurriest_node = i;
            }
        }
        if (hurriest_node == -1) break;
        // 插入最急的点 注意hurriest的id是not_insert的id
        auto &info_for_insert = pos_cost[hurriest_node][0];
        if (info_for_insert.cur_route >= routes.size()) throw runtime_error("out of range.");
        do_insert_from_info(routes[info_for_insert.cur_route], info_for_insert, not_insert[hurriest_node]);
        visited[hurriest_node] = true;

        // 重新计算剩余点的cost
        update_position_cost(not_insert, routes, pos_cost);
    }
    vector<int> nonInsertNew;
    for (int i = 0; i < num_of_nodes; ++i) {
        if (visited[i] == false) nonInsertNew.push_back(not_insert[i]);
    }
    not_insert = nonInsertNew;
#ifdef CAL_TIME
    time_t end_time = clock();
    cout << __func__ << " Run time is " << static_cast<double>(end_time - start) / CLOCKS_PER_SEC << endl;
#endif
}

void RegretInsertion::update_position_cost(
        std::vector<int> &NonInsert,
        std::vector<Route> &routes,
        std::vector<std::vector<InsertInfo>> &pos_cost) {
    for(auto &i : pos_cost)
        for (auto &j: i)
            j.cost = INF;

    for (int id_in_nonInsert = 0; id_in_nonInsert < NonInsert.size(); ++id_in_nonInsert) {
        if (visited[id_in_nonInsert]) continue; // 如果已经访问过的 就不更新
        // 否则对每个路径找到使得cost最小的插入位置 记录插入信息
        for (int route_id = 0; route_id < routes.size(); ++route_id) {
            for (int cur_pos = 1; cur_pos < routes[route_id].size(); ++cur_pos) {
                auto info = evaluate_insert_with_rs(routes[route_id], route_id, cur_pos, NonInsert[id_in_nonInsert]);
                //todo 这里可以用定长的priority_queue重构
                if (info.cost < pos_cost[id_in_nonInsert][route_id].cost) {
                    pos_cost[id_in_nonInsert][route_id] = info;
                }
            }
        }
    }
}

// 计算所有节点的regret_value, 将结果保存在regret_value中, 同时会破坏pos_cost的顺序
inline void RegretInsertion::calculate_regret_value(std::vector<std::vector<InsertInfo>> &pos_cost, std::vector<double> &regret_value) {
    for(int i = 0; i < regret_value.size(); ++i) {
        regret_value[i] = 0;
        if (!visited[i]) {
            sort(pos_cost[i].begin(), pos_cost[i].end(), RCLLess());
            for (int j = 1; j < k; ++j) {
                regret_value[i] += (pos_cost[i][j].cost - pos_cost[i][0].cost);
            }
        }
    }
}

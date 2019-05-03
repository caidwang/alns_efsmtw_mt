//
// Created by wsc on 19-5-2.
//

#include "SequentialNodeInsertion.h"
#include "VRP_Solution.h"
#include "logistics.hpp"
using namespace std;


void SequentialNodeInsertion::repairSolution(ISolution &sol) {
    auto &newS = dynamic_cast<VRP_Solution&>(sol);
    if (newS.getNonInserted().empty()) return ;
    auto &routes = newS.getRoutes();
    for (auto it: newS.getNonInserted()) {
        int cur_route, cur_position;
        double cur_cost;
        NodePostionRCL rcl(5);
        for (cur_route = 0; cur_route < routes.size(); ++cur_route) {
            int length = routes[cur_route].size(); // 插入位置可以从1到length-1
            for (cur_position = 1; cur_position < length; ++cur_position) {
                auto info = evaluate_insert_with_rs(routes[cur_route], cur_route, cur_position, it);
                rcl.push(info);
            }
        }
        auto best = rcl.randGet();
        do_insert_from_info(routes[cur_route], best, it);
    }
    newS.getNonInserted().clear();
//    assert(newS.isFeasible());
}

insert_info
SequentialNodeInsertion::evaluate_insert_with_rs(Route &route, int cur_route, int cur_position, int node_id) {
    insert_info info{cur_route, cur_position, -1, -1, 0};
    int sr_id_ahead = find_best_charger(cur_route, cur_position, node_id, true); //todo implement find_best_charger
    int sr_id_post = find_best_charger(cur_route, cur_position, node_id, false);
    info.cost = route.evaluateInsert(cur_position, node_id);
    double cost_p = route.evaluateInsert(cur_position, vector<int>{node_id, sr_id_post});
    double cost_a = route.evaluateInsert(cur_position, vector<int>{sr_id_ahead, node_id});
    double cost_both = route.evaluateInsert(cur_position, vector<int>{sr_id_ahead, node_id, sr_id_post});
    if (cost_a < info.cost) {
        info.RS_ahead = sr_id_ahead;
        info.cost = cost_a;
    }
    if (cost_p < info.cost) {
        info.RS_ahead = -1;
        info.RS_post = sr_id_post;
    }
    if (cost_both < info.cost) {
        info.RS_post = sr_id_post;
        info.RS_ahead = sr_id_ahead;
        info.cost = cost_both;
    }
    return info;
}

void SequentialNodeInsertion::do_insert_from_info(Route &route, insert_info &info, int node_id) {
    if (info.RS_post > 0) {
        route.insert(info.RS_post, info.cur_position);
    }
    route.insert(node_id, info.cur_position);
    if (info.RS_ahead > 0) {
        route.insert(info.RS_ahead, info.cur_position);
    }
}



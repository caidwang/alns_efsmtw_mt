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
        NodePositionRCL rcl(5);
        for (cur_route = 0; cur_route < routes.size(); ++cur_route) {
            int length = routes[cur_route].size(); // 插入位置可以从1到length-1
            for (cur_position = 1; cur_position < length; ++cur_position) {
                auto info = evaluate_insert_with_rs(routes[cur_route], cur_route, cur_position, it);
                rcl.push(info);
            }
        }
        auto best = rcl.randGet();
        if (best.cur_route >= routes.size()) throw runtime_error("out of range.");
        do_insert_from_info(routes[best.cur_route], best, it);
    }
    newS.getNonInserted().clear();
}



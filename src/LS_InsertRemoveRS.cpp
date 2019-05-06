//
// Created by wsc on 19-5-4.
//

#include <VRP_Solution.h>
#include "LS_InsertRemoveRS.h"
using namespace std;

bool LS_InsertRemoveRS::performLocalSearch(ISolution &sol) {
    auto &prevS = dynamic_cast<VRP_Solution&>(sol);
    bool ever_improved = false, improvement;
    do {
        improvement =false;
        int it = 0, best_SR, best_pos;
        double best_decrease = 0;
        Route *best_route = nullptr;
        bool best_is_insert;
        for (auto &route : prevS.getRoutes()) {
            for(int pos = 1; pos < route.size(); ++pos) {
                ++it;
                double delta = 0;
                bool prev_is_SR = false, move_is_insert;
                int SR = -1;
                // 如果当前点是SR 尝试删除它
                if (route.get_node_by_position(pos) > 1000) {
                    delta = route.evaluateRemove(pos);
                    move_is_insert = false;
                    prev_is_SR = true;
                }
                else {
                    // 如果当前点的前一点不是SR 尝试在它前面插入一个SR
                    if (!prev_is_SR) {
                        SR = find_best_charger(route, pos, route.get_node_by_position(pos), true, Route::get_time_mat());
                        delta = route.evaluateInsert(pos, SR);
                        move_is_insert = true;
                    }
                    prev_is_SR = false;
                }
                // 得到S', 其中f(S') < f(S)
                if (delta < best_decrease) {
                    ever_improved = true;
                    improvement = true;
                    best_decrease = delta;
                    best_route = &route;
                    best_is_insert = move_is_insert;
                    best_pos = pos;
                    best_SR = SR;
                }
                if (it >= 50 && improvement) break;
            }
        }

        // S = S'
        if (improvement) {
            if (best_is_insert) {
                best_route->insert(best_SR, best_pos);
            }
            else {
                best_route->remove(best_pos);
            }
        }
    } while (improvement);
    return ever_improved;
}

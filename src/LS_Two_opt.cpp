//
// Created by wsc on 19-5-2.
//

#include "LS_Two_opt.h"
using namespace std;
bool LS_Two_opt::performLocalSearch(ISolution &sol) {
    auto &prevS = dynamic_cast<VRP_Solution&> (sol);
    bool ever_improved = false, improvement;
    do {
        improvement = false;
//        double prevCost = prevS.getPenalizedObjectiveValue();
        double best_decrease = 0;
        Route* best_route = nullptr;
        int best_begin_pos, best_end_pos;
        int it = 0;
        for (auto &route : prevS.getRoutes()) {
            for (int begin_pos = 1; begin_pos < route.size()-2; ++begin_pos) {
                for (int end_pos = begin_pos + 1; end_pos < route.size()-1; ++end_pos) {
                    ++it;
                    double delta = route.evaluateTwoOpt(begin_pos, end_pos);
                    if (delta < best_decrease) {
                        ever_improved = true;
                        improvement = true;
                        best_decrease = delta;
                        best_route = &route;
                        best_begin_pos = begin_pos;
                        best_end_pos = end_pos;
                    }
                    if (it >= 50 && improvement) break;
                }
            }
        }
        if (improvement) {
            best_route->execTwoOpt(best_begin_pos, best_end_pos);
        }
        // todo update solution.
    } while (improvement);
    return ever_improved;
}

//
// Created by wsc on 19-5-4.
//

#include <VRP_Solution.h>
#include "LS_InterRelocate.h"
using namespace std;

bool LS_InterRelocate::performLocalSearch(ISolution &sol) {
    auto &prevS = dynamic_cast<VRP_Solution&> (sol);
    bool ever_improved = false, improvement;
    int cnt_improved = 0;
    do {
//        cout << cnt_improved << endl;
        improvement = false;
        Route *best_route_remove_from = nullptr, *best_route_insert_in = nullptr;
        double best_decrease = 0;
        int best_pos_remove_from, best_pos_insert_in;
        int it = 0;
        for (auto &route_from : prevS.getRoutes()) {
            for (int pos_from = 1; pos_from < route_from.size() - 1; ++pos_from) {
                for (auto &route_in: prevS.getRoutes()) {
                    for (int pos_in = 1; pos_in < route_in.size(); ++ pos_in) {
                        ++it;
                        if (&route_from == &route_in) continue; // todo check again
                        double delta = route_from.evaluateRemove(pos_from) + route_in.evaluateInsert(pos_in, route_from.route_seq[pos_from]);
                        if (delta < best_decrease) {
                            ever_improved = true;
                            improvement = true;
                            best_decrease = delta;
                            best_pos_insert_in = pos_in;
                            best_pos_remove_from = pos_from;
                            best_route_remove_from = &route_from;
                            best_route_insert_in = &route_in;
                        }
                        if (it >= 50 && improvement) break;
                    }
                }
            }

        }
        if(improvement) {
            best_route_insert_in->insert(best_route_remove_from->route_seq[best_pos_remove_from], best_pos_insert_in);
            best_route_remove_from->remove(best_pos_remove_from);
            ++cnt_improved;
        }
    } while(improvement && cnt_improved < 100);
    return ever_improved;
}

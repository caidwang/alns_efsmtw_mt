//
// Created by wsc on 19-5-1.
//

#include "RandomAndRelatedRemoval.h"
#include "VRP_Solution.h"
#include <vector>

#ifndef MIN_Q_RATE
#define MIN_Q_RATE 0.1
#define MAX_Q_RATE 0.15
#endif

using namespace std;

void RandomAndRelatedRemoval::destroySolution(ISolution &sol) {
    auto &vrp_s = dynamic_cast<VRP_Solution&>(sol);
    // todo Q的基数是customer好还是nodes好
    int randomQ = (rand() % static_cast<int>((MAX_Q_RATE - MIN_Q_RATE) * static_cast<double>(vrp_s.getNCustomers()))) + static_cast<int>(MIN_Q_RATE * static_cast<double>(vrp_s.getNCustomers()));

    double delete_rate = static_cast<double>(randomQ) / static_cast<double>(vrp_s.getNCustomers());
    int deleted = 0, cust_id = 0;
    vector<int> remove_list;

    while (deleted < randomQ) {
        // 等概率选择客户点, 对于被选中的点, 在与其相关性最高的5个点中随机选择一个点一并删除
        cust_id = 1 + (cust_id) % vrp_s.getNCustomers(); // 1-nCustomer
        if (static_cast<double>(rand()%1000) / 1000.0 < delete_rate) {
            bool not_in_list = true;
            for (auto it: remove_list) {
                if (it == cust_id) {
                    not_in_list = false;
                    break;
                }
            }
            if (not_in_list) {
                remove_list.push_back(cust_id);
                remove_list.push_back(relatedness.related_RCL(cust_id, 5));
                deleted += 2;
            }
        }
        ++cust_id;
    }
    vrp_s.remove(remove_list);
}
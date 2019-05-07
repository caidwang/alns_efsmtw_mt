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
    int cust_id = 0;
    vector<int> remove_list;
    set<int> remove_set;
    while (remove_set.size() < randomQ) {
        // 等概率选择客户点, 对于被选中的点, 在与其相关性最高的5个点中随机选择一个点一并删除
        ++cust_id; // 1-nCustomer
        if (cust_id > 1000) cust_id -= 1000;
        if (static_cast<double>(rand()%1000) < randomQ) {
            // 按照randomQ/nCustomers 的概率 删除cust_id
            if (remove_set.find(cust_id) == remove_set.end()) { // 如果cust_id不在remove_list中
                remove_set.insert(cust_id);
                remove_list.push_back(cust_id);
            }

            int most_related = relatedness.related_RCL(cust_id, 5); // 找到和当前点关系最紧密的5个点中的随机一个
            int cnt = 0;
            while (remove_set.find(most_related) != remove_set.end()) { // 如果该点已经在remove_list 中 重新选择
                if (cnt == 5) break; // 如果一直循环找不到 则不删除
                most_related = relatedness.related_RCL(cust_id, 5);
                ++cnt;
            }
            if (cnt < 5) {
                remove_set.insert(most_related);
                remove_list.push_back(most_related);
            }
        }

    }
    vrp_s.remove(remove_list);
}
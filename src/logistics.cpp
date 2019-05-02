#include "logistics.hpp"
#include "util.hpp"
#include <cassert>
using namespace std;


int N;

/**
 * Insert the node to the route at given position.
 * Execute without constraint check. 注意!!　执行过程不进行约束检查
 * @param x Node reference
 * @param position, the position in route to insert [1, route.size -1)
 */
void Route::insert(const Node &x, const int position) {
    if (position < 1) return ;
    auto it = route_seq.begin() + position; // 插入点前置点位置
    auto it_time = leave_time.begin() + position; // 插入点前置点位置
    auto it_dist = dist_rest.begin() + position; // 插入点前置点位置

    // 更新载重和容量约束
    if (x.type == 2) {
        cur_weight += x.demand_weight;
        cur_volume += x.demand_volume;
    }
    // 更新总里程变量
    total_dist += - dist_mat[*(it-1)][*it]  + dist_mat[*(it-1)][x.id] + dist_mat[x.id][*it];

//    int prior_point_dist = *(it_dist-1);
//
//    dist_rest.insert(it_dist, (*(it-1) > 1000) ?
//        vehicle.max_distance() - dist_mat[*(it-1)][*it] :
//        prior_point_dist - dist_mat[*(it-1)][x.id]); // 插入点到达时剩余里程
    route_seq.insert(it, x.id);

    dist_rest.insert(it_dist, 0);
    leave_time.insert(it_time, 0);
    // update route_seq
    // 插入后迭代器失效
//    it_dist = dist_rest.begin() + position;
//    it_time = leave_time.begin() + position;
//    it = route_seq.begin() + position;

    recalculate_time_and_dist_seq(position);
}

/**
 * Insert the node to the route at given position
 * O(n)复杂度
 * @param x the index of node in node_list
 * @param position
 */
void Route::insert(const int x, int position) {
    insert(node_list[x], position);
}

/**
 * Drop the node at position of route.
 * O(n)复杂度
 * No constraints check executes.
 * @param position, the position of node to drop [1, route.size -1)
 */
void Route::drop(int position) {
    if (route_seq.size() < 3 || position >= route_seq.size()) return; // 只有起终点或position非法

    auto it = route_seq.begin() + position - 1; // 被删除节点的前置点
    auto it_time = leave_time.begin() + position - 1; // 被删除节点的前置点dist_mat[*it]
    auto it_dist = dist_rest.begin() + position - 1; // 被删除节点的前置点

    const Node &node_to_drop = node_list[route_seq[position]]; // 记录被删除点的节点属性

    // 因删除节点产生的里程增加, 通常是负数
    total_dist += - dist_mat[*it][node_to_drop.id] - dist_mat[node_to_drop.id][*(it + 2)] + dist_mat[*it][*(it + 2)];

    if (node_to_drop.type == 2) {
        cur_weight -= node_to_drop.demand_weight;
        cur_volume -= node_to_drop.demand_volume;
    }

    route_seq.erase(it + 1);
    leave_time.erase(it_time + 1);
    dist_rest.erase(it_dist + 1);
//    bool meet_next_charger = false;

    recalculate_time_and_dist_seq(position);
}

/**
 * 对由于position位置的节点插入或删除, 导致后面节点的离开时间和剩余电量的变化进行更新
 * O(n)复杂度
 * @param position 新插入的点的位置或删除点的位置
 */
void Route::recalculate_time_and_dist_seq(int position) {
    auto it_seq = route_seq.begin() + position;
    auto it_time = leave_time.begin() + position;
    auto it_dist = dist_rest.begin() + position;
    for (; it_seq != route_seq.end() - 1; ++it_seq, ++it_time, ++it_dist) {
        *it_time = max(node_list[*it_seq].early_time, *(it_time-1) + time_mat[*(it_seq-1)][*it_seq]) + OPERATION_TIME; // 根据前一点的离开时间, 更新当前节点的离开时间

        *it_dist = (*it_seq > 1000) ? vehicle.max_distance() : *(it_dist -1) - dist_mat[*(it_seq-1)][*it_seq]; // 根据前一点的剩余距离和当前点是否是充电站, 更新当前节点的离开时间
    }
    *it_time = *(it_time-1) + time_mat[*(it_seq-1)][*it_seq]; // 返程到仓库的时间
    *it_dist = *(it_dist-1) - dist_mat[*(it_seq-1)][*it_seq]; // 返程时的剩余电量
}

/**
 * 将begin_pos到end_pos之间的节点序列进行反向
 * @param begin_pos 反转开始位置
 * @param end_pos 反转结束位置
 */
void Route::reverse(int begin_pos, int end_pos) {
    // TODO 要是换成list记得改成自带reverse方法
    auto it0 = route_seq.begin() + begin_pos;
    auto it1 = route_seq. begin() + end_pos;
    while (it0 != it1) {
        int temp = *it0;
        *it0 = *it1;
        *it1 = temp;
        ++it0;
        --it1;
    }
    recalculate_time_and_dist_seq(begin_pos);
}



///**
// * Check if the route has time violation
// * @param r the route
// * @return true if time violation occurs.
// *         false if no time violation occurs.
// */
//bool check_time_violation(const Route &r, int *violation_pos) {
//    if (r.leave_time.back() > depot.last_time) return true;
//    for (decltype(r.leave_time.size()) i = 1; i < r.leave_time.size() - 1; i++) {
//        if (r.leave_time[i] > node_list[r.route_seq[i]].last_time + OPERATION_TIME) {
//            if (violation_pos) *violation_pos = i;
//            return true;
//        }
//    }
//    return false;
//}
//
//bool check_time_violation(const Route &r, int customer, int position) {
//    return check_time_violation(r, node_list[customer], position);
//}
//
//bool check_time_violation(const Route &r, const Node &customer, int position) {
//    if (r.leave_time[position - 1] > customer.last_time) return true; // 离开上一个节点时就已经超过待插入节点的最晚时间
//    vector<int> node_index = r.route_seq;
//    vector<int> time = r.leave_time;
//    time.insert(time.begin() + position, 0);
//    node_index.insert(node_index.begin() + position, customer.id);
//    auto it_time = time.begin() + position;
//    auto it_index = node_index.cbegin() + position;
//    // todo 将插入点的it_time的计算和后继点分开, 以避免node_index的拷贝
//    for (; it_index != node_index.cend() - 1; ++it_index, ++it_time) {
//        *it_time = max(node_list[*it_index].early_time, *(it_time-1) + time_mat[*(it_index-1)][*it_index]) + OPERATION_TIME;
//        if (*it_time > node_list[*it_index].last_time + OPERATION_TIME) return true;
//    }
//    return *(it_time-1) + time_mat[*(it_index-1)][*it_index] > depot.last_time;
//}
//
//
///**
// * check if the route has capacity violation
// * @param r
// * @return true if violation occurs.
// *         false if no violation exists.
// */
//bool check_capacity_violation(const Route &r) {
//    return r.cur_volume > r.vehicle.capacity() || r.cur_weight > r.vehicle.max_weight();
//}
//
//bool check_capacity_violation(const Route &r , int customer, int position) {
//    return check_capacity_violation(r, node_list[customer], position);
//}
//
//bool check_capacity_violation(const Route &r , const Node &customer, int position) {
//    return customer.demand_volume + r.cur_volume > r.vehicle.capacity() || customer.demand_weight + r.cur_weight > r.vehicle.max_weight();
//}
//
///**
// * check if the route has distance violation
// * @param r
// * @return true if violation occurs.
// *         false if no violation exists.
// */
//bool check_distance_violation(const Route &r, int *violation_pos) {
//    auto it_route = r.route_seq.begin() + 1;
//    auto it_dist = r.dist_rest.begin() + 1;
//    for (; it_dist < r.dist_rest.end(); ++it_dist, ++it_route) {
//        if (*it_route > 1000) {
//            if (*(it_dist - 1) - dist_mat[*(it_route - 1)][*it_route] < 0) {
//                if (violation_pos) *violation_pos = it_route - r.route_seq.begin();
//                return true;
//            }
//
//        }
//        else if (*it_dist < 0) {
//            if (violation_pos) *violation_pos = it_route - r.route_seq.begin();
//            return true;
//        }
//    }
//    return false;
//}
//
//bool check_distance_violation(const Route &r, int customer, int position) {
//    return check_distance_violation(r, node_list[customer], position);
//}
//
//bool check_distance_violation(const Route &r, const Node &customer, int position) {
//    if (r.dist_rest[position - 1] < dist_mat[r.route_seq[position-1]][customer.id]) return true;
//    vector<int> node_index = r.route_seq;
//    vector<int> dist_rest = r.dist_rest;
//    dist_rest.insert(dist_rest.begin() + position, 0);
//    node_index.insert(node_index.begin() + position, customer.id);
//    auto it_index = node_index.cbegin() + position; // 指向插入点
//    auto it_dist = dist_rest.begin() + position; // 指向插入点
//    int max_distance = r.vehicle.max_distance();
//    for (; it_index < node_index.cend() - 1; ++it_index, ++it_dist) {
//        *it_dist = *(it_dist -1) - dist_mat[*(it_index-1)][*it_index];
//        if (*it_dist < 0) return true;
//        if (*it_index > 1000) *it_dist = max_distance;
//    }
//    return *(it_dist-1) + dist_mat[*(it_index-1)][*it_index] < 0;
//}
//
//
//
//void find_charger(Route &route, int &charger_index, int &charger_position, bool bin_direct) {
//    charger_index = -1;
//    int violation_pos = -1;
//    check_distance_violation(route, &violation_pos); // 找到发生dist_rest小于0(violation)的位置
//    if (violation_pos == -1) return ;
//    auto violation_prior = route.route_seq.begin() + violation_pos - 1; // 指向violation前一个点的index
//    int violation_prior_rest_dist = route.dist_rest[violation_pos - 1];
//    vector<int> available_charger; // 可以到达的所有充电站
//    for (int i = 1001; i <= 1100; i++) {
//        if (dist_mat[*violation_prior][i] <= violation_prior_rest_dist)
//            available_charger.push_back(i);
//    }
//    int min_dist = INT32_MAX;
//    for (auto index : available_charger) {
//        // 注意防范violation出现在第一个点和第二个点之间的情况, 只能在前一个点之后加, 但是如果violation出现在倒数第一个点, 没有问题
//        if (bin_direct && violation_pos > 1 && !check_distance_violation(route, index, violation_pos-1) && !check_time_violation(route, index, violation_pos-1)) {
//            int delta = -dist_mat[*(violation_prior-1)][*violation_prior] + dist_mat[*(violation_prior-1)][index] + dist_mat[index][*violation_prior];
//            if (delta < min_dist) {
//                charger_index = index;
//                charger_position = violation_pos - 1;
//                min_dist = delta;
//            }
//        }
//
//        if (!check_distance_violation(route, index, violation_pos) && !check_time_violation(route, index, violation_pos)) {
//            int delta = -dist_mat[*violation_prior][*(violation_prior+1)] + dist_mat[*violation_prior][index] + dist_mat[index][*(violation_prior + 1)];
//            if (delta < min_dist) {
//                charger_index = index;
//                charger_position = violation_pos;
//                min_dist = delta;
//            }
//        }
//    }
//}
//
//void remove_redundant_charger(Route &route, int position) {
//    int charger_index;
//    for (++position; position < route.route_seq.size(); ++position) {
//        if (route.route_seq[position] > 1000) {
//            charger_index = route.route_seq[position];
//            route.drop(position);
//            if (check_distance_violation(route))
//                route.insert(charger_index, position);
//        }
//    }
//}
//
//bool last_time_order(const Node &n, const Node &v) {
//    return n.last_time < v.last_time;
//}
//bool early_time_order(const Node &n, const Node &v) {
//    return n.early_time < v.early_time;
//}

void test_logistic() {
    // preprocess for test
    vector<vector<int>> dist_mat;
    int n = 6;
    for (int i = 0; i < n; ++i) {
        dist_mat.emplace_back();
        for (int j = 0; j < n; ++j) {
            dist_mat[i].push_back(1000);
        }
    }
//    dist_mat
//    {{1000, 1000, 1000, 1000, 1000, 1000,},
//        {1000, 1000, 1000, 1000, 1000, 1000,},
//        {1000, 1000, 1000, 1000, 1000, 1000,},
//        {1000, 1000, 1000, 1000, 1000, 1000,},
//        {1000, 1000, 1000, 1000, 1000, 1000,},
//        {1000, 1000, 1000, 1000, 1000, 1000,},}
    vector<vector<int>> time_mat;
    for (int i = 0; i < n; ++i) {
        time_mat.emplace_back();
        for (int j = 0; j < n; ++j) {
            time_mat[i].push_back(60);
        }
    }
//    time_mat
//    {{60, 60, 60, 60, 60, 60,},
//        {60, 60, 60, 60, 60, 60,},
//        {60, 60, 60, 60, 60, 60,},
//        {60, 60, 60, 60, 60, 60,},
//        {60, 60, 60, 60, 60, 60,},
//        {60, 60, 60, 60, 60, 60,},}
    vector<Node> node_list;
    node_list.emplace_back();
    node_list.emplace_back(1, 2, 0.1, 0.1, 480, 1440);
    node_list.emplace_back(2, 2, 0.1, 0.1, 480, 1440);
    node_list.emplace_back(3, 2, 0.1, 0.1, 480, 1440);
    node_list.emplace_back(4, 3, 0, 0, 0, 1440);
    Route::set_graph_info(dist_mat, time_mat, node_list);


}

void test_route_insert() {
    Route route0(1, 1, 480);
    route0.insert(1, 1);
    route0.insert(2, 2);
    vector<int> time_seq0 {480, 570, 660, 720};
    vector<int> dist_seq0 {100000, 99000, 98000, 97000};
    assert(route0.leave_time == time_seq0);
    assert(route0.dist_rest == dist_seq0);

}
void test_route_remove() {
    Route route(1, 1, 480);

}
void test_route_drop() {
    Route route(1, 1, 480);

}
Relatedness::Relatedness(const vector<Node> &node_list,
        const vector<std::vector<int>> &dist_mat, 
        const vector<std::vector<int>> &time_mat) :
        dist_mat(dist_mat), time_mat(time_mat), node_list(node_list) {
    int N = node_list.size();
    for (int i = 0; i < N; ++i) {
        relatedness_table.emplace_back();
        for (int j = 0; j < N; ++j) {
            if (i == j)
                relatedness_table[i][j] = INT32_MAX;
            else if (j < i)
                relatedness_table[i][j] = relatedness_table[j][i];
            else 
                relatedness_table[i][j] = static_cast<int>(distance_weight*dist_mat[i][j]) +
                        static_cast<int>(max(0, node_list[j].early_time - (node_list[i].last_time + OPERATION_TIME + time_mat[i][j])) * waiting_time_weight) +
                        static_cast<int>(max(0, node_list[i].early_time + OPERATION_TIME + time_mat[i][j] - node_list[j].last_time) * time_window_weight);
        }
    }
}
int Relatedness::get_relatedness(int node1, int node2) const {
    return relatedness_table[node1][node2];
}

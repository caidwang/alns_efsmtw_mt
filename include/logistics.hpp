# ifndef LOGISTICS_H
# define LOGISTICS_H
# include <vector>
# include <queue>
# include <string>
# include <cmath>
# include<algorithm>
# include <iostream>
# include<fstream>
#ifndef INF
#define INF 1000000
#endif

// Edge object
//extern std::vector<std::vector<int> > time_mat;
//extern std::vector<std::vector<int> > dist_mat;
//extern std::vector<int> charger_near_customer;

// Node object
const int OPERATION_TIME = 30; // same for customer and charge_station.
const int CHARGE_COST = 50; // 一次
const int WAITING_COST = 24; // 元/h
class Node {
public:
    int id;
    int type;
    float demand_weight{0}, demand_volume{0};
    int early_time, last_time;
    Node(): id(0), type{1}, early_time(480), last_time(1440) {};
    explicit Node(int id): Node() {this->id = id; };
    Node(int id, int type, float w, float v, int et, int lt): id(id), type(type), demand_volume(v), demand_weight(w), early_time(et), last_time(lt){ }
};

// vehicle object
const int IVECO = 1; // vehicle type
const int TRUCK = 2; // vehicle type

class Vehicle {
private:
        int type, v_id;
public:
    Vehicle(int id, int v_type) : v_id(id), type(v_type) { }
    explicit Vehicle(int id) : v_id(id), type(IVECO) { }
public:
    int get_vehicle_id() const { return v_id; };
    int get_type() const {return type; }
    int capacity() const;
    double max_weight() const;
    int max_distance() const;
    int cost_pkm() const;
    int fix_cost() const;
};

int inline Vehicle::capacity() const {
    return type == IVECO? 12 : 16;
}
double inline Vehicle::max_weight() const{
    return type == IVECO? 2.0 : 2.5;
}
int inline Vehicle::max_distance() const{
    return type == IVECO? 100000: 120000;
}
int inline Vehicle::cost_pkm() const{
    return type == IVECO? 12 : 14;
}
int inline Vehicle::fix_cost() const{
    return type == IVECO? 200 : 300;
}

class Route {
public:
    Vehicle vehicle;
    std::vector<int> route_seq; // 路线经过的节点的id序列
    // TODO 下面两个是否需要
    std::vector<int> leave_time; // 离开第i个节点的时间, 最后一个节点的时间记录返程到达仓库的时间
    std::vector<int> dist_rest; // 离开第i个节点时的剩余里程
    double cur_weight, cur_volume;
    int total_dist;
    Route(int v_id, int v_type, int start_time) : vehicle(v_id, v_type), cur_weight(0), cur_volume(0), total_dist(0) {
        route_seq.push_back(0);
        route_seq.push_back(0);
        leave_time.push_back(start_time);
        leave_time.push_back(start_time);
        dist_rest.push_back(vehicle.max_distance());
        dist_rest.push_back(vehicle.max_distance());
    }
    size_t size() const {return route_seq.size(); }
    int get_node_by_position(int position);
    double ACUT();
    void insert(int id, int position);
//    void drop(int position);
    void remove(int position);
    void execTwoOpt(int begin, int end);
    double evaluateInsert(int positon, int node_id); // 需要考虑插入客户点和插入RS两种情况
    double evaluateInsert(int position, std::vector<int> node_seq);
    double evaluateRemove(int position); // 需要考虑移除RS和移除客户点两种情况
    double evaluateTwoOpt(int begin, int end); // begin和end是反转序列的开始点和结束点
    //evaluate 系列方法给的是动作发生前后后路径penalizedCost的变化值
//    void reverse(int begin_pos, int end_pos);
    static void set_graph_info(std::vector<std::vector<int>> &d_mat, std::vector<std::vector<int>> &t_mat, std::vector<Node> &nl) {
        time_mat = t_mat;
        node_list = nl;
        dist_mat = d_mat;
    }

private:
    static std::vector<std::vector<int>> dist_mat;
    static std::vector<std::vector<int>> time_mat;
    static std::vector<Node> node_list;
    void insert(const Node &x, int position); // deprecate
    void recalculate_time_and_dist_seq(int position);

};


// 记录插入node点时, 尝试前后充电站信息
struct InsertInfo {
    int cur_route;
    int cur_position;
    int RS_ahead;
    int RS_post;
    double cost;
    bool change_type;
};

// 用于RCL的比较函数, RCL中, 最大的总是在最前, 当有更小的加入时, 最大的被pop出
class RCLLess {
    bool operator()(const InsertInfo &x, const InsertInfo &y) {return x.cost < y.cost; }
};

// 在插入中使用的RCL 保存的是插入cost最小的前max_number个插入动作的InsertInfo
class NodePostionRCL {
public:
    explicit NodePostionRCL(int max_number) : maxNumber(max_number) { }
    void push(InsertInfo item) {
        RCL.push(item);
        if (RCL.size() > maxNumber) RCL.pop();
    }
    InsertInfo randGet() {
        int index = max(rand() % maxNumber, RCL.size() -1);
        while (index) {
            RCL.pop();
            --index;
        }
        return RCL.top();
    }
private:
    int maxNumber;
    std::priority_queue<InsertInfo, RCLLess> RCL;
};

// 针对每个节点在每个位置的插入, 进行四种尝试 {v}, {f,v}, {v, g}, {f, v, g}
// 返回带惩罚的cost最小的插入方式的info
InsertInfo evaluate_insert_with_rs(Route &route, int cur_route, int cur_position, int node_id);

// 与evaluate_insert_with_rs一起使用, 在插入时按照info的信息, 确定实际插入时如何处理RS
void do_insert_from_info(Route &route, InsertInfo &info, int node_id);

/**
 * ahead为true时, 返回node_id与cur_position-1节点之间插入的最优充电站
 * ahead为false时, 返回node_id与cur_position节点之间插入的最优充电站
 */
int find_best_charger(Route& cur_route, int cur_position, int node_id, bool ahead);
//bool check_time_violation(const Route &, int *violation_pos = nullptr);
//bool check_time_violation(const Route &, int customer, int position);
//bool check_time_violation(const Route &r, const Node &customer, int position);
//bool check_capacity_violation(const Route &);
//bool check_capacity_violation(const Route &r, int customer, int position = 0);
//bool check_capacity_violation(const Route &r, const Node &customer, int position = 0);
//bool check_distance_violation(const Route &, int *violation_pos = nullptr);
//bool check_distance_violation(const Route &, int customer, int position);
//bool check_distance_violation(const Route &, const Node &customer, int position);


//void find_charger(Route &route, int &charger_index, int &charger_position, bool bin_direct_search=true);
//void remove_redundant_charger(Route &, int);
//bool last_time_order(const Node &, const Node &);
//bool early_time_order(const Node &, const Node &);

# endif //LOGISTICS_H

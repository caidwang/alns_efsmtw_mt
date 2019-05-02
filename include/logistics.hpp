# ifndef LOGISTICS_H
# define LOGISTICS_H
# include <vector>
# include <string>
# include <cmath>
# include<algorithm>
# include <iostream>
# include<fstream>
// get_relatedness parameters
# define DISTANCE_WEIGHT 1
# define WAITING_TIME_WEIGHT 0.2
# define TIME_WINDOW_WEIGHT 1

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

    void insert(int id, int position);
    void drop(int position);
    void reverse(int begin_pos, int end_pos);
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


class Relatedness {
public:
    Relatedness(const std::vector<Node> &node_list,
            const std::vector<std::vector<int>> &dist_mat,
            const std::vector<std::vector<int>> &time_mat);
    int get_relatedness(int node1, int node2) const;
private:
    const double distance_weight = DISTANCE_WEIGHT, waiting_time_weight = WAITING_TIME_WEIGHT, time_window_weight=TIME_WINDOW_WEIGHT;
    const std::vector<std::vector<int>> &dist_mat;
    const std::vector<std::vector<int>> &time_mat;
    const std::vector<Node> &node_list;
    std::vector<std::vector<int>> relatedness_table;
};

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

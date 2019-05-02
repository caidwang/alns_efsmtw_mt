#include "util.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
using namespace std;
static const int N = 1101;
static const float lng_depot = 52423.045871259856;
static const float lat_depot = 28796.746932550333;
static const double PI = 3.14159265;

/**
 * read distance and travel time between nodes, save them to the global matrix
 * dist_mat and time_mat
 * guarantee that the size of dist_mat and time_mat is [N][N]
 */
void read_dist_time_mat(const string &dist_mat_file, 
        std::vector<std::vector<int>> &dist_mat, 
        std::vector<std::vector<int>> &time_mat, int nnode) {
    ifstream in(dist_mat_file);
    if (!in) throw invalid_argument("dist mat reading fails.\n");
    dist_mat.resize(nnode);
    time_mat.resize(nnode); 
    for (int i = 0; i < nnode; i++) {
        dist_mat[i].resize(nnode);
        time_mat[i].resize(nnode);
        dist_mat[i][i] = 0;
        time_mat[i][i] = 0;
    }
    bool has_header =true;
    int data[5];
    char c;
    string value;
    while(getline(in, value)) {
        if (has_header) {
            has_header = false;
            continue;
        }
        istringstream readvalue(value);
        readvalue >> data[0] >> c
                  >> data[1] >> c
                  >> data[2] >> c
                  >> data[3] >> c
                  >> data[4];
        
        dist_mat[data[1]][data[2]] = data[3];
        time_mat[data[1]][data[2]] = data[4];
    }
}

/**
 * read node information from file.
 * for all Nodes: id, type={1,2,3}, lng, lat, time is recorded in minutes.
 * angle is the angle between line that links node to depot and north.
 * for depot and charge station: demand_weight = demand_capacity = 0
 * for charge station: early_time = 0, last_time = 24*60
 */
void read_nodes(const string &filename, vector<Node> &nodes, bool header) {
    ifstream in(filename);
    string line;
    while (getline(in, line)) {
        if (header) {
            header = false;
            continue;
        }
        istringstream data(line);
        int id, type, eh, em, lh, lm, et, lt;
        float lng, lat, w, v;
        char c;
        data >> id >> c
            >> type >> c
            >> lng >> c
            >> lat >> c;
        if (!(data >> w >> c >> v >> c)) { // w,v, -> -,-,
            w = 0; v = 0;
            data.clear(); // reset the stream state to good bit. remember!!!
            data >> c >> c >> c;
        }
        if (!(data >> eh >> c >> em >> c >> lh >> c >> lm)){
            et = 0; lt = 1440;
        }
        else {
            et = eh * 60 + em;
            lt = lh * 60 + lm;
            if (et > lt) lt = 1440;
        }
        nodes.emplace_back(id, type, w, v, et, lt);
    }
}

// function for inspect the dist_mat and time_mat
// 打印最后5行
void print_mat(vector<vector<int>> &mat) {
    for (auto i = mat.end() - 5; i != mat.end(); i++) {
        for (auto j = (*i).end()-5; j != (*i).end(); j++) cout << *j << " ";
        cout << endl;
    }
}

// function for inspect vector<Node>, range is [begin, end)
void print_node_list(vector<Node> &list, int begin, int end) {
    for (auto i = list.begin() + begin; i != list.begin() + end; i++){
        cout << (*i).id << " "
        << setprecision(4) << (*i).demand_weight << " "
        << setprecision(4) << (*i).demand_volume << " "
        << (*i).early_time << " "
        << (*i).last_time << endl;
    }
}



// 打印格式
// vehicle_id, vehicle_type, id->id->id, start_time, back_time, total_dist
void print_solution(const VRP_Solution &solution, ostream &out) {
    const auto &route_list = solution.getRoutes();
#ifndef NDEBUG
    cout << "in print_solution, number of routes is " << route_list.size() << endl;
#endif
    for (const auto &route : route_list) {
        bool start = true;
        out << route.vehicle.get_vehicle_id() << "," << route.vehicle.get_type() << ",";

        for (auto pos : route.route_seq) {
            if (start) {
                out << pos;
                start = false;
            }
            else
                out <<  "->" << pos;
        }
        out << "," << route.leave_time.front() << ","
        << route.leave_time.back() << ","
        << route.total_dist << ","
        << "0" // waiting time.
        << endl;
    }
}
// 打印结果 形式为第一行 总路径数n,
// 之后n行 车辆类型 车辆id 路径载重率 路径容量率 id->id->id
void analyse_result(const vector<Route> &route_list) {
    cout << route_list.size() << endl;
    for (const auto &route : route_list) {
        cout << route.vehicle.get_type() << "\t" << route.vehicle.get_vehicle_id() << "\t";
        cout << route.cur_weight / route.vehicle.max_weight() * 100 << "%\t"
             << route.cur_volume / route.vehicle.capacity() * 100 << "%\t";
        bool start = true;
        for (auto pos : route.route_seq) {
            if (start) {
                cout << pos;
                start = false;
            }
            else
                cout <<  "->" << pos;
        }
        cout << endl;
    }
}


int cal_total_cost(const vector<Route> &route_list, vector<vector<int>> &dist_mat, vector<vector<int>> &time_mat, vector<Node> &node_list) {
    int total = 0, waiting_time;
    for (const auto &route : route_list) {
        total += route.vehicle.fix_cost();
        total += route.vehicle.cost_pkm() * route.total_dist / 1000;
        waiting_time = 0;
        for (int position = 1; position < route.route_seq.size() - 1; ++position) {
            if (route.route_seq[position] > 1000) total += CHARGE_COST;
            waiting_time += max(0, node_list[route.route_seq[position+1]].early_time -  route.leave_time[position] - time_mat[route.route_seq[position]][route.route_seq[position+1]]);
        }
        total += waiting_time * WAITING_COST / 60;
    }
    return total;
}

// calculate the angle between line that links node to depot and north.
float cal_angle(float lng, float lat) {
    float delta_y = lng - lng_depot;
    float delta_x = lat - lat_depot;
    if (delta_x < 0.0001 && delta_x > -0.0001 && delta_y < 0.0001 && delta_y > -0.0001) return 0.0;
    float angle = atan(delta_y / delta_x) + PI / 2 ;
    if (delta_x > 0) angle += PI;
    return angle;
}

void test_cal_angle() {
    cout << cal_angle(52500, 28793) << endl;
    cout << cal_angle(52300, 28793) << endl;
    cout << cal_angle(52300, 28800) << endl;
    cout << cal_angle(52500, 28800) << endl;
}

void print_test_route(const Route &route) {
    int n = route.route_seq.size();
    cout << "route: (vehicle:" << route.vehicle.v_id << " type: " << route.vehicle.max_distance() << ") " << endl;
    cout << "route sequence :" << endl;
    for (auto item : route.route_seq) {
        cout << item << "->";
    }
    cout << endl;
    cout << "leave time :" << endl;
    for (auto item : route.leave_time) {
        cout << item << "->";
    }
    cout << endl;
    cout << "distance rest :" << endl;
    for (auto item : route.dist_rest) {
        cout << item << "->";
    }
    cout << endl;
}

void test_insert_and_drop() {
    cout << "1. Pure customer insert" << endl;
    Route r(0, IVECO, 8*60);
    r.insert(node_list[10], 1);
    print_test_route(r);
    cout << "1.1 Pure customer insert" << endl;
    r = Route(0, IVECO, 8*60);
    r.insert(node_list[10], 1);
    r.insert(node_list[20], 1);
    r.insert(node_list[30], 2);
    r.insert(node_list[60], 3);
    r.insert(node_list[70], 4);
    print_test_route(r);

    cout << "-----------------------------------" << endl;
    cout << "2. customer insert and RS insert" << endl;
    r = Route(0, IVECO, 8*60);
    r.insert(node_list[10], 1);
    r.insert(node_list[20], 1);
    r.insert(node_list[30], 2);
    r.insert(node_list[1050], 3);
    r.insert(node_list[60], 4);
    r.insert(node_list[70], 4);
    print_test_route(r);
    cout << "-----------------------------------" << endl;
    cout << "3. customer insert and RS insert 1" << endl;
    r = Route(0, IVECO, 8*60);
    r.insert(node_list[10], 1);
    r.insert(node_list[20], 1);
    r.insert(node_list[30], 2);
    r.insert(node_list[1050], 2);
    r.insert(node_list[60], 2);
    r.insert(node_list[70], 2);
    r.insert(node_list[1010], 4);
    print_test_route(r);
    cout << "-----------------------------------" << endl;
    cout << "4. customer insert and drop" << endl;
    cout << "before insert" << endl;
    r = Route(0, IVECO, 8*60);
    r.insert(node_list[10], 1);
    r.insert(node_list[20], 1);
    r.insert(node_list[30], 2);
    r.insert(node_list[60], 3);
    r.insert(node_list[70], 4);
    print_test_route(r);
    cout << "after insert" << endl;
    r.insert(node_list[40], 3);
    print_test_route(r);
    cout << "after drop" << endl;
    r.drop(3);
    print_test_route(r);
    cout << "-----------------------------------" << endl;
    cout << "5. SR insert and drop" << endl;
    cout << "before insert" << endl;
    r = Route(0, IVECO, 8*60);
    r.insert(node_list[10], 1);
    r.insert(node_list[20], 1);
    r.insert(node_list[30], 2);
    r.insert(node_list[60], 3);
    r.insert(node_list[70], 4);
    print_test_route(r);
    cout << "after insert" << endl;
    r.insert(node_list[1010], 3);
    print_test_route(r);
    cout << "after drop" << endl;
    r.drop(3);
    print_test_route(r);
}


void test_check_time_violation() {
    Route r(0, 0, depot.early_time);
    r.insert(261, 1);
    r.insert(1100, 2);
    r.insert(16, 3);
    r.insert(567, 4);
    r.insert(361, 5);
    r.insert(443, 6);
    r.insert(711, 7);
    print_test_route(r);
    cout <<"测试check_time_violation(r): 预测 0 实际 " << check_time_violation(r) << endl;
    cout <<"测试check_time_violation(r, cus, pos): 预测 1 实际 " << check_time_violation(r, 20, 8) << endl;
    cout <<"测试check_time_violation(r, cus, pos): 预测 1 实际 " << check_time_violation(r, 21, 2) << endl;
    int violation = 0;
    r.insert(21, 2);
    check_time_violation(r, &violation);
    cout <<"测试check_time_violation(r, vio_pos): 实际插入后重新检查 预测 4 实际 "<< violation << endl;
}

void test_check_distance_violation() {
    Route r(0, 0, depot.early_time);
    r.insert(261, 1);
    r.insert(1100, 2);
    r.insert(16, 3);
    r.insert(567, 4);
    r.insert(361, 5);
    r.insert(443, 6);
    r.insert(711, 7);
    print_test_route(r);
    cout << "测试测试check_dist_violation(r) 预测 0 实际 " << check_distance_violation(r) << endl;
    r.drop(2);
    cout << "测试测试check_dist_violation(r) 预测 1 实际 " << check_distance_violation(r) << endl;
    int violation_pos = -1;
    check_distance_violation(r, &violation_pos);
    cout << "测试测试check_dist_violation(r) 预测 7 实际 " << violation_pos << endl;
    cout << "测试测试check_dist_violation(r) 预测 0 实际 " << check_distance_violation(r, 1100, 2) << endl;
}

void test_check_capacity_violation() {

}

void test_find_charger() {
    Route r(0, 0, depot.early_time);
    r.insert(261, 1);
    r.insert(16, 2);
    r.insert(567, 3);
    r.insert(361, 4);
    r.insert(443, 5);
    r.insert(711, 6);
    int violation_pos = -1;
    check_distance_violation(r, &violation_pos);
    cout << "出现dist_violation的位置 预测3  实际 " << violation_pos << endl;
    int charger_index, charger_pos;
    print_test_route(r);
    find_charger(r, charger_index, charger_pos);
    cout << "测试find_charger: charger_index " << charger_index << " charger_position " << charger_pos << endl;
    r.insert(charger_index, charger_pos);
    cout << r.total_dist << endl;
    cout << "插入后dist_violation: " << check_distance_violation(r) << " 插入后time_violation: " << check_time_violation(r) << endl;
}




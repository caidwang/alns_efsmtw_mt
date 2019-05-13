#include "util.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <unistd.h>

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
void print_solution(VRP_Solution &solution, ostream &out, bool debug) {
    auto &route_list = solution.getRoutes();
    if (debug) {
        cout << "in print_solution, number of routes is " << route_list.size()
             << ", the solution is " << (solution.isFeasible() ? "feasible." : "unfeasible.") << endl;
    }
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
        out << "," << route.get_start_time() << ","
        << route.get_back_time() << ","
        << route.total_dist << ","
        << route.get_waiting_time() // waiting time.
        << endl;
    }
}
// 打印结果 形式为第一行 总路径数n,
// 之后n行 车辆类型 车辆id 路径载重率 路径容量率 id->id->id
void analyse_result(const vector<Route> &route_list) {
    cout << route_list.size() << endl;
    for (const auto &route : route_list) {
        cout << route.vehicle.get_type() << "\t" << route.vehicle.get_vehicle_id() << "\t";
        cout << route.total_weight / route.vehicle.max_weight() * 100 << "%\t"
             << route.total_volume / route.vehicle.capacity() * 100 << "%\t";
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

bool lastest_modified_file(const char *dir_name,string & name) {
    DIR *dirp;
    struct dirent *dp;
    char pwd[200];
    getcwd(pwd, 200);
    chdir(dir_name);
    bool file_exist = false;
    dirp = opendir(dir_name);
    timespec mtime{0,0};

    while ((dp = readdir(dirp)) != NULL) {
        struct stat stat_buf;
        lstat(dp->d_name, &stat_buf);
//        cout << dp->d_name << stat_buf.st_mtim.tv_sec << endl;
        if (dp->d_type != DT_DIR && stat_buf.st_ctim.tv_sec > mtime.tv_sec) {
            file_exist =true;
            name = dp->d_name;
            mtime = stat_buf.st_mtim;
        }
    }
    (void) closedir(dirp);
    chdir(pwd);
    return file_exist;
}

void write_answer(ISolution *sol, const std::string &path) {
    time_t now = time(nullptr);
    tm *local = localtime(&now);
    char writeFileName[200];
    sprintf(writeFileName, "solution_%02d_%02d_%02d_%02d", local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
    ofstream fw(path + writeFileName);
    if (fw) {
        print_solution(dynamic_cast<VRP_Solution &>(*sol), fw, false);
    }
}
//
//int cal_total_cost(const vector<Route> &route_list, vector<vector<int>> &dist_mat, vector<vector<int>> &time_mat, vector<Node> &node_list) {
//    int total = 0, waiting_time;
//    for (const auto &route : route_list) {
//        total += route.vehicle.fix_cost();
//        total += route.vehicle.cost_pkm() * route.total_dist / 1000;
//        waiting_time = 0;
//        for (int position = 1; position < route.route_seq.size() - 1; ++position) {
//            if (route.route_seq[position] > 1000) total += CHARGE_COST;
//            waiting_time += max(0, node_list[route.route_seq[position+1]].early_time -  route.leave_time[position] - time_mat[route.route_seq[position]][route.route_seq[position+1]]);
//        }
//        total += waiting_time * WAITING_COST / 60;
//    }
//    return total;
//}





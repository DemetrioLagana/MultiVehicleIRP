/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: demetrio
 *
 * Created on 31 marzo 2022, 9.15
 */

#include <cstdlib>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include "Build_MVIRP_graph.hpp"
#include "ExtremeDelivery.hpp"
#include "Routes_Generator.hpp"

using namespace std;
using namespace mvirp;
/*
 * 
 */
int main(int argc, char** argv) {
//    cout << argv[1] << endl;
    string dir_instances(argv[1]);
    int first_pos, last_pos;
    string filepath_instance, dir, dir_VRP, name_instance;
    DIR *dp_instances;
    struct dirent *dmvirp_instances;
    struct stat filestat;
    stringstream ss;
    ss.str("");
    ss << dir_instances << "_RESULTS";
    int index_str = dir_instances.find("MVIRP", 0);
    if (index_str > 0) {
        first_pos = dir_instances.find_last_of("/");
        ss.str("");
        ss.clear();
        ss << dir_instances.substr(first_pos + 1) << "_RESULTS";
    }
    dir = ss.str();
    if (mkdir(&(dir[0]), 0777) == -1)
        cout << "Warning: " << strerror(errno) << endl;
    dp_instances = opendir(dir_instances.c_str());
    if (dp_instances == NULL) {
        cout << "Error(" << errno << ") opening " << dir_instances << endl;
        return errno;
    }
    while (dmvirp_instances = readdir(dp_instances)) {
        name_instance = dmvirp_instances->d_name;
//        cout << name_instance << endl;
        last_pos = name_instance.find(".");
        name_instance = name_instance.substr(0, last_pos);
        filepath_instance = dir_instances + "/" + dmvirp_instances->d_name;
        if (stat(filepath_instance.c_str(), &filestat))
            continue;
        if (S_ISDIR(filestat.st_mode))
            continue;
        MVIRP_graph* Graph = new MVIRP_graph();
        read_ABS_Instances(filepath_instance.c_str(), dir, Graph);
        dir_VRP = Graph->VRP_folder;
        if (mkdir(&(dir_VRP[0]), 0777) == -1)
            cout << "Warning: " << strerror(errno) << endl;
        if (Graph->print_graph_info) {
            Graph->print_V();
            Graph->print_E();
            vector<set<int> > T_sets = Graph->get_T_sets();
        }
        ExtremeDelivery_model* model = new ExtremeDelivery_model(*Graph);
        model->solve_iteratively();
        vector<vector<list<std::tuple<int, Vertex, float> > > > p_maps = model->get_period_maps();
        Routes_Generator* generator = new Routes_Generator(Graph);
        generator->get_VRP_solutions(p_maps);
        generator->printRoutes();
        delete Graph;
        delete model;
        delete generator;
    }
    return 0;
}


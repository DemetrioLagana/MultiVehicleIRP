/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Build_MVIRP_graph.hpp
 * Author: demetrio
 *
 * Created on 31 marzo 2022, 11.36
 */

#ifndef BUILD_MVIRP_GRAPH_HPP
#define BUILD_MVIRP_GRAPH_HPP

#include "Definitions.hpp"
#include "ConfigFile.hpp"

using namespace std;

namespace mvirp {
    class MVIRP_graph {
    public:
        bool print_graph_info;
        int flipping_threshold, num_times_solve_model, num_vehicles, Q, H, num_vertices, tabu_tenure;
        
        UndirectedGraph* graph;
        ConfigFile* cfg;
        ofstream* MVIRP_logfile;
        string Problem_name;
        string Instance_name;
        string VRP_folder;
        
        MVIRP_graph();
        void build_edges_and_set_vertices();
//        void compute_I0s_bards();
//        void compute_U_Tplus_Tminus();
        vector<set<int> > get_T_sets ();
        void print_V();
        void print_E();
        ~MVIRP_graph();
    private:
        void compute_I0s_bards();
        void compute_U_Tplus_Tminus();
    };
    
    
//    class Build_MVIRP_graph {
//    public:
//        Build_MVIRP_graph();
//        Build_MVIRP_graph(const Build_MVIRP_graph& orig);
//        virtual ~Build_MVIRP_graph();
//    private:
//
//    };
    
    void read_ABS_Instances(const char *filename, string dir, MVIRP_graph* G);
}

#endif /* BUILD_MVIRP_GRAPH_HPP */


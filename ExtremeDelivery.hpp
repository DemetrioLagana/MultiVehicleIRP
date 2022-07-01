/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Quantities_model.hpp
 * Author: demetrio
 *
 * Created on 9 aprile 2022, 10.01
 */

#ifndef QUANTITIES_MODEL_HPP
#define QUANTITIES_MODEL_HPP

#include "Build_MVIRP_graph.hpp"
#include <tuple>

using namespace std;

namespace mvirp {

    class ExtremeDelivery_model {
    public:
        bool export_model;
        int counter, id_model, flipping_threshold, MIP_emphasis, num_times_solve_model, probing, tabu_tenure, time, var_sel;
        MVIRP_graph* G;
        string LP;
        vector<std::tuple<int, int, int, float > > z_q_sol;
        typedef vector<std::tuple<int, int, int, float > > zq_sol;
        list<pair<int, zq_sol> > zq_solutions;
        
        
        ENV env;
        MODEL model;
        CPLEX cplex;
        THREE_INDEX_VAR q, y;
        TWO_INDEX_VAR z;
        RANGE_CONSTRAINTS rng;
        RANGE_FLIPPING_CONSTRAINTS flrng;
        ExtremeDelivery_model(MVIRP_graph &Graph);
        bool solve();
        void solve_iteratively();
        vector<vector<list<std::tuple<int, Vertex, float> > > > get_period_maps();
        
        ~ExtremeDelivery_model();
    private:
        list<IloRange> Flipping_constraints;
        void define_variables();
        void define_constraints();
        void define_objective();
        void set_parameters_solver();
        IloNum const build_model();
        bool get_z_q_sol();
        bool find_zq_sol (zq_sol& first, zq_sol& second);
        bool zq_sol_is_in_zq_solutions(zq_sol& zq);
        bool update_zq_solutions (zq_sol& zq);
        list<std::tuple<int, Vertex, float> > get_p_map(int &p, zq_sol& z_q_sol);
        void print_solution();
        void clear_and_restart_the_model();
    };
}

#endif


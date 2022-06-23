/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Route_Generator.hpp
 * Author: demetrio
 *
 * Created on 10 maggio 2022, 16.19
 */

#ifndef ROUTE_GENERATOR_HPP
#define ROUTE_GENERATOR_HPP

#include "Build_MVIRP_graph.hpp"
#include "Route.hpp"
#include <VRPH.h>

extern "C" {
#include <concorde.h>
}

using namespace std;

namespace mvirp {

    class Routes_Generator {
    public:
        bool use_CONCORDE;
        static int counter;
        MVIRP_graph* G;
        list<Route> Routes;
        Routes_Generator(MVIRP_graph* &graph, list<Route> routes);
        Routes_Generator(MVIRP_graph* &graph);
        virtual ~Routes_Generator();
        bool addRoute(Route& r);
        void printRoutes();
        void get_VRP_solutions (vector<vector<list<std::tuple<int, Vertex, float> > > >& p_maps);
    private:
        bool routeExists(Route &r);
        string createVRP_file(int &time, list<std::tuple<int, Vertex, float> >& data);
        float applyTSP (int &time, list<Vertex>& vertices);
        void solve_VRP (int &time, list<std::tuple<int, Vertex, float> >& data);
    };
}

#endif /* ROUTE_GENERATOR_HPP */


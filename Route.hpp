/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Route.hpp
 * Author: demetrio
 *
 * Created on 29 aprile 2022, 13.21
 */

#ifndef ROUTE_HPP
#define ROUTE_HPP

#include "Definitions.hpp"

using namespace std;

namespace mvirp {

    class Route {
    public:
        static int counter;
        Route();
        Route(list<Vertex> trip, double weight);
        Route(const Route& other) : Id_Route(other.Id_Route), route(other.route), cost(other.cost) {}
        
        ~Route();
        
        void addVertexToRoute(Vertex v);
        void setIdRoute(int Id_route);
        void setRoute(list<Vertex> trip);
        void setCost(double weight);
        int getNumServed();
        bool findVertex(Vertex v, UndirectedGraph* &graph);
        int getVertexPosition(Vertex v, UndirectedGraph* &graph);
        list<Vertex> getRoute();
        list<Vertex> getServed(UndirectedGraph* &graph);
        int getIdRoute();
        double getCost();
        double getSavingtoRemove(Vertex remove,UndirectedGraph* &graph);
        void computeCost(UndirectedGraph* &G);
        void printRoute(ofstream* log, UndirectedGraph* &graph);
        bool isEqual(Route &r, UndirectedGraph* &graph);
        bool routeIsInSets(list<Route> set,UndirectedGraph* &graph);
        void addTime(int t);
    private:
        set<int> activated_times;
        int Id_Route;
        list<Vertex> route;
        float cost;
        Vertex getVertexbyID(int Id, const UndirectedGraph* &graph);
    };
}

#endif /* ROUTE_HPP */


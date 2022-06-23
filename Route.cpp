/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Route.cpp
 * Author: demetrio
 * 
 * Created on 29 aprile 2022, 13.21
 */

#include "Route.hpp"

namespace mvirp {

    int Route::counter = 1;

    Route::Route() {
        Id_Route = Route::counter++;
        cost = 0;
    }
    
    Route::Route(list<Vertex> trip, double weight) {
        Id_Route = Route::counter++;
        route = trip;
        cost = weight;
    }
    
    Route::~Route() {
        route.clear();
    }
    void Route::addVertexToRoute(Vertex n) {
        route.push_back(n);
    }

    void Route::setIdRoute(int Id_route) {
        Id_Route = Id_route;
    }
    
    void Route::setRoute(list<Vertex> trip) {
        route = trip;
    }

    void Route::setCost(double weight) {
        cost = weight;
    }
    
    Vertex Route::getVertexbyID(int Id, const UndirectedGraph* &graph) {
        Vertex v;
        MapVertexIndex Id_Vertex = get(vertex_index, *graph);
        vertex_range_t Vp = vertices(*graph);
        for (; Vp.first != Vp.second; ++Vp.first)
            if (Id_Vertex[*Vp.first] == Id)
                return *(Vp.first);
        return v;
    }
    
    bool Route::isEqual(Route &r, UndirectedGraph* &graph) {
        MapVertexIndex Vertex_Index = get(vertex_index, *graph);
        list<Vertex> r_this = getRoute();
        list<Vertex> r_v = r.getRoute();
        if (r_this.size() != r_v.size())
            return false;
        else {
            bool check1 = true;
            list<Vertex>::iterator it1, it2;
            it2 = r_v.begin();
            for (it1 = r_this.begin(); it1 != r_this.end(); ++it1) {
                if (Vertex_Index[*it1] != Vertex_Index[*it2]) {
                    check1 = false;
                    break;
                }
                ++it2;
            }
            if (check1)
                return check1;

            bool check2 = true;
            r_v.reverse();
            it2 = r_v.begin();
            for (it1 = r_this.begin(); it1 != r_this.end(); ++it1) {
                if (Vertex_Index[*it1] != Vertex_Index[*it2]) {
                    check2 = false;
                    break;
                }
                ++it2;
            }
            if (check2)
                return check2;
            else
                return false;

        }
        return true;
    }
    
    bool Route::routeIsInSets(list<Route> set, UndirectedGraph* &graph) {
        for (Route &r : set)
            if (this->isEqual(r, graph))
                return true;
        return false;
    }
    
    void Route::addTime(int t) {
        activated_times.emplace(t);
    }
    
    int Route::getNumServed() {
        return (getRoute().size() - 2);
    }
    
    int Route::getVertexPosition(Vertex v, UndirectedGraph* &graph) {
        int pos = 0;
        MapVertexIndex Vertex_Index = get(vertex_index, *graph);
        for (list<Vertex>::iterator it = route.begin(); it != route.end(); it++) {
            if (Vertex_Index[((*it))] == Vertex_Index[v])
                return pos;
            pos++;
        }
        return -1;
    }
    
    bool Route::findVertex(Vertex v, UndirectedGraph* &graph) {
        MapVertexIndex Vertex_Index = get(vertex_index, *graph);
        for (list<Vertex>::iterator it = route.begin(); it != route.end(); it++) {
            if (Vertex_Index[((*it))] == Vertex_Index[v])
                return true;
        }
        return false;
    }
    
    list<Vertex> Route::getServed(UndirectedGraph* &graph) {
        list<Vertex> output;
        MapVertexIndex Vertex_Index = get(vertex_index, *graph);
        for (list<Vertex>::iterator it = route.begin(); it != route.end(); it++) {
            if (Vertex_Index[((*it))] != 0)
                output.push_back(*it);
        }
        return output;
    }
    
    void Route::computeCost(UndirectedGraph* &graph) {
        MapEdgeCost Edge_Cost = get(edge_cost_t(), *graph);
        MapVertexIndex Vertex_Index = get(vertex_index, *graph);
        list<Vertex>::iterator i, j;
        j = route.begin();
        ++j;
        for (i = route.begin(); j != route.end() & i != route.end(); ++i) {
            if (Vertex_Index[*i] <= Vertex_Index[*j])
                cost += Edge_Cost[edge(*i, *j, *graph).first];
            else
                cost += Edge_Cost[edge(*j, *i, *graph).first];
            ++j;
        }
    }
    
    double Route::getSavingtoRemove(Vertex remove, UndirectedGraph* &graph) {
        double c = 0;
        MapEdgeCost Edge_Cost = get(edge_cost_t(), *graph);
        MapVertexIndex Vertex_Index = get(vertex_index, *graph);
        list<Vertex>::iterator i, j;
        j = route.begin();

        for (i = route.begin(); j != route.end() & i != route.end(); ++i) {
            if (Vertex_Index[*i] == Vertex_Index[remove]) {
                j = i;
                --j;
                if (Vertex_Index[*i] > Vertex_Index[*j])
                    c += Edge_Cost[edge(*j, *i, *graph).first];
                else
                    c += Edge_Cost[edge(*i, *j, *graph).first];
                j = i;
                ++j;
                if (Vertex_Index[*i] > Vertex_Index[*j])
                    c += Edge_Cost[edge(*j, *i, *graph).first];
                else
                    c += Edge_Cost[edge(*i, *j, *graph).first];
                j = i;
                j--;
                Vertex j1 = *j;
                j = i;
                ++j;
                Vertex j2 = *j;
                if (Vertex_Index[j1] > Vertex_Index[j2])
                    c -= Edge_Cost[edge(j2, j1, *graph).first];
                else
                    c -= Edge_Cost[edge(j1, j2, *graph).first];
            }
        }
        return c;
    }
    
    list<Vertex> Route::getRoute() {
        return route;
    }

    double Route::getCost() {
        return cost;
    }

    int Route::getIdRoute() {
        return Id_Route;
    }
    
    void Route::printRoute(ofstream* log, UndirectedGraph* &graph) {
        MapVertexIndex Vertex_Index = get(vertex_index, *graph);
        if (log != NULL) {
            (*log) << "Id route = " << Id_Route << " <";
            for (list<Vertex>::iterator it = route.begin(); it != route.end(); it++){
                if (std::next(it) != route.end()) {
                    (*log) << " " << Vertex_Index[(*it)] + 1 << " --> ";
                } else {
                    (*log) << " " << Vertex_Index[(*it)] + 1 << ">" << " - cost = " << cost << endl;
                }
            }
        } else {
            cout << "Id route = " << Id_Route << " <";
            for (list<Vertex>::iterator it = route.begin(); it != route.end(); it++){
                if (std::next(it) != route.end()) {
                    cout << " " << Vertex_Index[(*it)] + 1 << " --> ";
                } else {
                    cout << " " << Vertex_Index[(*it)] + 1 << ">" << " - cost = " << cost << endl;
                }
            }
        }
    }
}


/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Build_MVIRP_graph.cpp
 * Author: demetrio
 * 
 * Created on 31 marzo 2022, 11.36
 */

#include <numeric>

#include "Build_MVIRP_graph.hpp"

namespace mvirp {
    
    MVIRP_graph::MVIRP_graph() {
        Problem_name = "";
        graph = new UndirectedGraph();
        cfg = new ConfigFile("config.cfg");
        
        Q = 0;
        H = 0;
        num_vehicles = cfg->getValueOfKey<int>("NUM_VEHICLES");
        num_vertices = 0;
        print_graph_info = cfg->getValueOfKey<bool>("PRINT_GRAPH_INFO");
    }
    
    
    MVIRP_graph::~MVIRP_graph() {
        if (MVIRP_logfile != NULL) {
            (*MVIRP_logfile).close();
            delete MVIRP_logfile;
            MVIRP_logfile = NULL;
        }
        //        cout << "after delete graph" << endl;
        vertex_iter v_first, v_second, v_next;
        vertex_range_t V = vertices(*graph);
        v_first = V.first;
        v_second = V.second;
        for (v_next = v_first; v_first != v_second; v_first = v_next) {
            ++v_next;
            clear_vertex(*v_first, *graph);
        }
        v_first = V.first;
        v_second = V.second;
        for (v_next = v_first; v_first != v_second; v_first = v_next) {
            ++v_next;
            remove_vertex(*v_first, *graph);
        }
        graph->clearing_graph();
        //        cout << "graph" << endl;
        delete graph;
        delete cfg;
        cfg = NULL;
    }
    
    void MVIRP_graph::build_edges_and_set_vertices() {
        MapEdgeName EdgeName = get(edge_name, *graph);
        MapEdgeIndex EdgeIndex = get(edge_index, *graph);
        MapEdgeCost EdgeCost = get(edge_cost_t(), *graph);
        int edge_index = 1;
        MapVertexIndex VertexIndex = get(vertex_index, *graph);
        MapVertexCoordX VertexCoordX = get(vertex_coordx_t(), *graph);
        MapVertexCoordY VertexCoordY = get(vertex_coordy_t(), *graph);
        stringstream ss;
        vertex_range_t Vp = vertices(*graph); 
        for (vertex_iter it = Vp.first; it != Vp.second; ++it) {
            vertex_iter it_next = it + 1;
            while (it_next != Vp.second) {
                Edge e = add_edge(*it, *it_next, *graph).first;
                ss.str("");
                ss.clear();
                ss << "(" << to_string(VertexIndex[*it] + 1) << "," << to_string(VertexIndex[*it_next] + 1) << ")";
                EdgeName[e] = ss.str();
                EdgeIndex[e] = edge_index++;
                EdgeCost[e] = round(sqrt(pow((VertexCoordX[*it_next] - VertexCoordX[*it]),2) + pow((VertexCoordY[*it_next] - VertexCoordY[*it]),2)));
                it_next++;
            }
        }
        compute_I0s_bards();
        compute_U_Tplus_Tminus();
    }
    
    void MVIRP_graph::print_V() {
        MapVertexName VertexName = get(vertex_name, *graph);
        MapVertexIndex VertexIndex = get(vertex_index, *graph);
        MapVertexCoordX VertexCoordX = get(vertex_coordx_t(), *graph);
        MapVertexCoordY VertexCoordY = get(vertex_coordy_t(), *graph);
        MapVertexInv0 VertexInv0 = get(vertex_inv0_t(), *graph);
        MapVertexProduction VertexProd = get(vertex_production_t(), *graph);
        MapVertexInvMax VertexInvMax = get(vertex_maxinv_t(), *graph);
        MapVertexInvMin VertexInvMin = get(vertex_mininv_t(), *graph);
        MapVertexDemand VertexDemand = get(vertex_demand_t(), *graph);
        MapVertexInvCost VertexInvCost = get(vertex_invcost_t(), *graph);
        
        MapVertexI0s I0s = get(vertex_inv0remaining_t(), *graph);
        MapVertexBarDs BarDs = get(vertex_demandremaining_t(), *graph);
        MapVertexTplus VertexTplus = get(vertex_Tplus_t(), *graph);
        MapVertexU VertexU = get(vertex_demandupperbound_t(), *graph);
        MapVertexTminus VertexTminus = get(vertex_Tminus_t(), *graph);
        vertex_range_t Vp = vertices(*graph); 
        if (MVIRP_logfile != NULL) {
            for (; Vp.first != Vp.second; ++Vp.first) {
                if (VertexName[*Vp.first] == "supplier") {
                    (*MVIRP_logfile) << "Node : " << VertexName[*Vp.first] << " | Index = " << VertexIndex[*Vp.first] + 1 <<
                            " | COORD_X = " << VertexCoordX[*Vp.first] << " | COORD_Y = " << VertexCoordY[*Vp.first] <<
                            " | INV0 = " << VertexInv0[*Vp.first] << " | PROD = " << VertexProd[*Vp.first] <<
                            " | INVCOST = " << VertexInvCost[*Vp.first] << endl;
                } else {
                    (*MVIRP_logfile) << "Node : " << VertexName[*Vp.first] << " | Index = " << VertexIndex[*Vp.first] + 1 <<
                            " | COORD_X = " << VertexCoordX[*Vp.first] << " | COORD_Y = " << VertexCoordY[*Vp.first] <<
                            " | INV0 = " << VertexInv0[*Vp.first] << " | MAXINV = " << VertexInvMax[*Vp.first] <<
                            " | MININV = " << VertexInvMin[*Vp.first] <<  " | DEMANDS = " << VertexDemand[*Vp.first] <<
                            " | INVCOST = " << VertexInvCost[*Vp.first] << " | I0s = " << I0s[*Vp.first] <<
                            " | bards = [" << BarDs[*Vp.first] << "] | Tplus = " << VertexTplus[*Vp.first] <<
                            " | Tminus = " << VertexTminus[*Vp.first] << " | U = " << VertexU[*Vp.first] << endl;
                }
            }
        }
    }
    
    void MVIRP_graph::print_E() {
        MapEdgeName EdgeName = get(edge_name, *graph);
        MapEdgeIndex EdgeIndex = get(edge_index, *graph);
        MapEdgeCost EdgeCost = get(edge_cost_t(), *graph);
        edge_range_t Ep = edges(*graph);
        if (MVIRP_logfile != NULL) {
            for (; Ep.first != Ep.second; ++Ep.first) {
                (*MVIRP_logfile) << "Edge : " << EdgeName[*Ep.first] << " | Index = " <<
                        EdgeIndex[*Ep.first] << " | Cost = " << EdgeCost[*Ep.first] << endl; 
            }
        }
    }
    
    void MVIRP_graph::compute_I0s_bards() {
        MapVertexInv0 VertexInv0 = get(vertex_inv0_t(), *graph);
        MapVertexDemand VertexDemand = get(vertex_demand_t(), *graph);
        
        MapVertexI0s I0s = get(vertex_inv0remaining_t(), *graph);
        MapVertexBarDs BarDs = get(vertex_demandremaining_t(), *graph);
        vertex_range_t Vp = vertices(*graph);
        Vp.first++;
        for (; Vp.first != Vp.second; ++Vp.first) {
            I0s[*Vp.first] = {VertexInv0[*Vp.first]};
            BarDs[*Vp.first] = {0};
            for (int t = 1; t <= H; ++t) {
                vector<int>::iterator it = VertexDemand[*Vp.first].begin() + t;
                if (t == H)
                    it = VertexDemand[*Vp.first].end();
                I0s[*Vp.first].emplace_back(max(0,VertexInv0[*Vp.first] - accumulate(VertexDemand[*Vp.first].begin(), it, 0)));
                BarDs[*Vp.first].emplace_back(max(0, VertexDemand[*Vp.first][t - 1] - I0s[*Vp.first][t - 1]));
            }
        }
    }
    
    void MVIRP_graph::compute_U_Tplus_Tminus() {
        MapVertexInvMax VertexInvMax = get(vertex_maxinv_t(), *graph);
        MapVertexDemand VertexDemand = get(vertex_demand_t(), *graph);
        MapVertexI0s I0s = get(vertex_inv0remaining_t(), *graph);
        MapVertexBarDs BarDs = get(vertex_demandremaining_t(), *graph);
        
        MapVertexTplus VertexTplus = get(vertex_Tplus_t(), *graph);
        MapVertexU VertexU = get(vertex_demandupperbound_t(), *graph);
        MapVertexTminus VertexTminus = get(vertex_Tminus_t(), *graph);
        
        vertex_range_t Vp = vertices(*graph);
        Vp.first++;
        for (; Vp.first != Vp.second; ++Vp.first) {
            for (int t = 1; t <= H; ++t) {
                vector<int> Tplus, u;
                if (BarDs[*Vp.first][t] > 0) {
                    Tplus.emplace_back(t);
                    u.emplace_back(min(BarDs[*Vp.first][t], VertexInvMax[*Vp.first] - I0s[*Vp.first][t - 1]));
                }
                for (int s = t + 1; s <= H + 1; ++s) {
                    vector<int>::iterator it_init = VertexDemand[*Vp.first].begin() + (t - 1);
                    vector<int>::iterator it_end = VertexDemand[*Vp.first].begin() + (s - 1);
                    if (s < H + 1 && BarDs[*Vp.first][s] > 0 && accumulate(it_init, it_end, 0) < VertexInvMax[*Vp.first]) {
                        Tplus.emplace_back(s);
                        u.emplace_back(min(BarDs[*Vp.first][s], VertexInvMax[*Vp.first] - accumulate(it_init, it_end, 0) - I0s[*Vp.first][s - 1]));
                    }
                    if (s == H + 1 && accumulate(it_init, VertexDemand[*Vp.first].end(), 0) < VertexInvMax[*Vp.first]) {
                        Tplus.emplace_back(s);
                        u.emplace_back(VertexInvMax[*Vp.first] - accumulate(it_init, it_end, 0) - I0s[*Vp.first][s - 1]);
                    }
                }
                VertexU[*Vp.first].emplace_back(u);
                VertexTplus[*Vp.first].emplace_back(Tplus);
            }
            for (int t = 1; t <= H; ++t) {
                vector<int> Tminus;
                for (int s = 1; s <= H; ++s) {
                    if (find(VertexTplus[*Vp.first][s - 1].begin(), VertexTplus[*Vp.first][s - 1].end(), t) != VertexTplus[*Vp.first][s - 1].end()) {
                        Tminus.emplace_back(s);
                    }
                }
                VertexTminus[*Vp.first].emplace_back(Tminus);
            }
        }
    }
    
    vector<set<int> > MVIRP_graph::get_T_sets() {
        vector<set<int> > T_sets;
        MapVertexIndex VertexIndex = get(vertex_index, *graph);
        MapVertexTminus VertexTminus = get(vertex_Tminus_t(), *graph);
        set<int> ids;
        for (int t = 1; t <= H; ++t) {
            set<int> t_set;
            vertex_range_t Vp = vertices(*graph);
            Vp.first++;
            for (; Vp.first != Vp.second; ++Vp.first) {
                for (int s = 1; s <= H; ++s) {
                    if (find(VertexTminus[*Vp.first][s - 1].begin(), VertexTminus[*Vp.first][s - 1].end(), t) != VertexTminus[*Vp.first][s - 1].end()) {
                        t_set.emplace(VertexIndex[*Vp.first]);
                        ids.emplace(VertexIndex[*Vp.first]);
                    }
                }
            }
            T_sets.emplace_back(t_set);
        }
        if (ids.size() != num_vertices - 1){
            assert(false);
        }
//        (*MVIRP_logfile) << T_sets << endl;
        return T_sets;
    }

//    Build_MVIRP_graph::Build_MVIRP_graph() {
//    }
//
//    Build_MVIRP_graph::Build_MVIRP_graph(const Build_MVIRP_graph& orig) {
//    }
//
//    Build_MVIRP_graph::~Build_MVIRP_graph() {
//    }
    
    void read_ABS_Instances(const char* filename, string dir, MVIRP_graph* G) {
        MapVertexName VertexName = get(vertex_name, *(G->graph));
        MapVertexCoordX VertexCoordX = get(vertex_coordx_t(), *(G->graph));
        MapVertexCoordY VertexCoordY = get(vertex_coordy_t(), *(G->graph));
        MapVertexInv0 VertexInv0 = get(vertex_inv0_t(), *(G->graph));
        MapVertexProduction VertexProd = get(vertex_production_t(), *(G->graph));
        MapVertexInvMax VertexInvMax = get(vertex_maxinv_t(), *(G->graph));
        MapVertexInvMin VertexInvMin = get(vertex_mininv_t(), *(G->graph));
        MapVertexDemand VertexDemand = get(vertex_demand_t(), *(G->graph));
        MapVertexInvCost VertexInvCost = get(vertex_invcost_t(), *(G->graph));
        
        ifstream source_file(filename);
        string extended_name(filename), instance_name, line, s;
        stringstream ss;
        
        int first_pos = extended_name.find_last_of("/");
        int last_pos = extended_name.find_last_of(".");
        instance_name = extended_name.substr(first_pos + 1, (last_pos - first_pos - 1));
        ss.str("");
        ss << instance_name << "m" << G->num_vehicles;
        G->Instance_name = ss.str();
        
        ss.str("");
        ss << "./" << dir << "/" << instance_name << "m" << G->num_vehicles << ".log";
        G->MVIRP_logfile = new ofstream(&(ss.str()[0]));
        ss.str("");
        ss << "./" << dir << "/" << instance_name << "m" << G->num_vehicles;
        G->Problem_name = ss.str();
        ss.str("");
        ss << "./" << dir << "/" << instance_name << "m" << G->num_vehicles << "_VRP_FILES";
        G->VRP_folder = ss.str();
        
        if (source_file.is_open()) {
            int line_counter = 0;
            while (source_file.good()) {
                line.clear();
                getline(source_file, line);
                line = trim(line);
                line = reduce(line);
//                cout << "line.size() = " << line.size() << " - " << line << endl;
                if (line.size() > 1 && line_counter == 0) {
                    ss.str("");
                    ss.clear();
//                    line = trim(line);
//                    line = reduce(line);
                    ss << line;
                    getline(ss, s, ' ');
                    G->num_vertices = atoi(&(s[0]));
                    for(int i = 0; i < G->num_vertices; ++i){
                        Vertex v = add_vertex(*G->graph);
                        if (i == 0)
                            VertexName[v] = "supplier";
                        else
                            VertexName[v] = "customer";
                    }
                    getline(ss, s, ' ');
                    G->H = atoi(&(s[0]));
                    getline(ss, s, ' ');
                    G->Q = round(atof(&(s[0]))/G->num_vehicles);
                }
                if (line.size() > 1 && line_counter == 1) {
                    ss.str("");
                    ss.clear();
//                    line = trim(line);
//                    line = reduce(line);
                    ss << line;
                    getline(ss, s, ' ');
                    Vertex supplier = vertex(atoi(&(s[0])) - 1, (*G->graph));
                    getline(ss, s, ' ');
                    VertexCoordX[supplier] = atof(&(s[0]));
                    getline(ss, s, ' ');
                    VertexCoordY[supplier] = atof(&(s[0]));
                    getline(ss, s, ' ');
                    VertexInv0[supplier] = atoi(&(s[0]));
                    getline(ss, s, ' ');
                    vector<int> prod(G->H, atoi(&(s[0])));
                    VertexProd[supplier] = prod;
                    getline(ss, s, ' ');
                    VertexInvCost[supplier] = atof(&(s[0]));
                }
                if (line.size() > 1 && line_counter > 1) {
                    ss.str("");
                    ss.clear();
//                    line = trim(line);
//                    line = reduce(line);
                    ss << line;
                    getline(ss, s, ' ');
                    Vertex customer = vertex(atoi(&(s[0])) - 1, (*G->graph));
                    getline(ss, s, ' ');
                    VertexCoordX[customer] = atof(&(s[0]));
                    getline(ss, s, ' ');
                    VertexCoordY[customer] = atof(&(s[0]));
                    getline(ss, s, ' ');
                    VertexInv0[customer] = atoi(&(s[0]));
                    getline(ss, s, ' ');
                    VertexInvMax[customer] = atoi(&(s[0]));
                    getline(ss, s, ' ');
                    VertexInvMin[customer] = atoi(&(s[0]));
                    getline(ss, s, ' ');
                    vector<int> demands(G->H, atoi(&(s[0])));
                    VertexDemand[customer] = demands;
                    getline(ss, s, ' ');
                    VertexInvCost[customer] = atof(&(s[0]));
                }
                if (line.size() > 1)
                    line_counter++;
            }
            source_file.close();
            cout << "Instance read successfully" << endl;
            G->build_edges_and_set_vertices();
        } else {
            cout << "Unable to open file" << endl;
        }
    }
}


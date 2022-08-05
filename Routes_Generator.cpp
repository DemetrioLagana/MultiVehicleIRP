/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Route_Generator.cpp
 * Author: demetrio
 * 
 * Created on 10 maggio 2022, 16.19
 */

#include "Routes_Generator.hpp"

namespace mvirp {

    Routes_Generator::Routes_Generator(MVIRP_graph* &graph, list<Route> routes) {
        counter = 1;
        G = graph;
        Routes = routes;
        use_CONCORDE = graph->cfg->getValueOfKey<bool>("USE_CONCORDE_WITHIN_VRPH");
    }

    Routes_Generator::Routes_Generator(MVIRP_graph* &graph) {
        counter = 1;
        G = graph;
        use_CONCORDE = graph->cfg->getValueOfKey<bool>("USE_CONCORDE_WITHIN_VRPH");
    }

    //    Route_Generator::Route_Generator(const Route_Generator& orig) {
    //    }
    //

    Routes_Generator::~Routes_Generator() {
        Routes.clear();
    }

    bool Routes_Generator::routeExists(Route& r) {
        for (auto & route : Routes)
            if (route.isEqual(r, G->graph))
                return true;
        return false;
    }

    bool Routes_Generator::addRoute(Route& r) {
        if (routeExists(r)) {
            return false;
        } else {
            r.setIdRoute(Routes.size() + 1);
            Routes.emplace_back(r);
            return true;
        }
    }

    void Routes_Generator::printRoutes() {
        (*G->MVIRP_logfile) << "ROUTES_PRINT_SECTION" << endl;
        (*G->MVIRP_logfile) << "Use TSP CONCORDE to optimize each route: " << use_CONCORDE << endl;
        for (auto & route : Routes) {
            route.printRoute(G->MVIRP_logfile, G->graph);
        }
    }

    float Routes_Generator::applyTSP(int &time, list<Vertex>& vertices) {
        MapVertexIndex Vertex_Index = get(vertex_index, *(G->graph));
        MapEdgeCost Edge_Cost = get(edge_cost_t(), *(G->graph));
        vector<Vertex> _S;
        list<Vertex> path;
        Route r = Route();
        // UPDATE 05.08.2022
        int rval = 0;
        for(list<Vertex>::iterator it = vertices.begin(); it != prev(vertices.end()); ++it) {
            _S.emplace_back(*it);
        }
//        for (auto & v : vertices) {
//            _S.emplace_back(v);
//        }
//        for (const auto& v : _S) {
//            (*G->MVIRP_logfile) << Vertex_Index[v] + 1 << " ";
//        }
//        (*G->MVIRP_logfile) << "_S.size() = " << _S.size() << endl;
        int cost = 0;

        if (_S.size() == 2) {
            path.push_back(_S[0]);
            path.push_back(_S[1]);
            path.push_back(_S[0]);
            cost = Edge_Cost[edge(_S[0], _S[1], *(G->graph)).first] * 2;
            r.setRoute(path);
            r.setCost(cost);
            r.addTime(time);
            addRoute(r);
            return cost;
        }

        int dim = 0;
        for (int i = 1; i < _S.size(); i++) {
            dim += i;
        }
        // UPDATE 04.08.2022
//        int dim = _S.size();
//        (*G->MVIRP_logfile) << "dim = " << dim << endl;
        // UPDATE 04.08.2022
//        int ecount = (dim * (dim - 1)) / 2;
        int *elist = new int[2 * dim];
        int *elen = new int[dim];
        // UPDATE 04.08.2022
//        int *elist = new int[2 * ecount];
//        int *elen = new int[ecount];
        int k = 0;
        //Building elist and elen
        for (int i = 0; i < _S.size(); i++) {
            for (int l = i + 1; l < _S.size(); l++) {
                elist[2 * k] = i;
                elist[2 * k + 1] = l;
                if (Vertex_Index[_S[i]] < Vertex_Index[_S[l]]) {
                    elen[k] = (int) Edge_Cost[edge(_S[i], _S[l], *(G->graph)).first];
                } else {
                    elen[k] = (int) Edge_Cost[edge(_S[l], _S[i], *(G->graph)).first];
                }
                k++;
            }
        }
        // UPDATE 04.08.2022
//        int edgeindex = 0;
//        int edgeWeight = 0;
//        for (int i = 0; i < _S.size(); i++) {
//            for (int l = i + 1; l < _S.size(); l++) {
//                elist[edgeindex] = i;
//                (*G->MVIRP_logfile) << "(elist[" << edgeindex << "] = " << elist[edgeindex] << ", ";
//                elist[edgeindex + 1] = l;
//                (*G->MVIRP_logfile) << "elist[" << edgeindex + 1 << "] = " << elist[edgeindex + 1] << ") - ";
//                if (Vertex_Index[_S[i]] < Vertex_Index[_S[l]]) {
//                    elen[edgeWeight] = (int) Edge_Cost[edge(_S[i], _S[l], *(G->graph)).first];
//                    (*G->MVIRP_logfile) << "elen[" << edgeWeight << "] = " << elen[edgeWeight] << endl;
//                } else {
//                    elen[edgeWeight] = (int) Edge_Cost[edge(_S[l], _S[i], *(G->graph)).first];
//                    (*G->MVIRP_logfile) << "elen[" << edgeWeight << "] = " << elen[edgeWeight] << endl;
//                }
//                edgeWeight++;
//                edgeindex = edgeindex + 2;
//            }
//        }
        if (_S.size() == 3) {
            cost = elen[0] + elen[1] + elen[2];
            path.push_back(_S[0]);
            path.push_back(_S[1]);
            path.push_back(_S[2]);
            path.push_back(_S[0]);

            delete elist;
            delete elen;

            r.setRoute(path);
            r.setCost(cost);
            r.addTime(time);
            addRoute(r);
            return cost;
        }
        if (_S.size() == 4) {
            int sum_path1 = elen[0] + elen[3] + elen[5] + elen[2];
            int sum_path2 = elen[0] + elen[4] + elen[5] + elen[1];
            int sum_path3 = elen[1] + elen[3] + elen[4] + elen[2];

            if (sum_path1 <= sum_path2 && sum_path1 <= sum_path3) {
                delete elist;
                delete elen;

                cost = sum_path1;
                path.push_back(_S[0]);
                path.push_back(_S[1]);
                path.push_back(_S[2]);
                path.push_back(_S[3]);
                path.push_back(_S[0]);

                r.setRoute(path);
                r.setCost(cost);
                r.addTime(time);
                addRoute(r);
                return cost;
            } else if (sum_path2 <= sum_path1 && sum_path2 <= sum_path3) {
                delete elist;
                delete elen;

                cost = sum_path2;
                path.push_back(_S[0]);
                path.push_back(_S[1]);
                path.push_back(_S[3]);
                path.push_back(_S[2]);
                path.push_back(_S[0]);

                r.setRoute(path);
                r.setCost(cost);
                r.addTime(time);
                addRoute(r);
                return cost;
            } else {
                delete elist;
                delete elen;

                cost = sum_path3;
                path.push_back(_S[0]);
                path.push_back(_S[2]);
                path.push_back(_S[1]);
                path.push_back(_S[3]);
                path.push_back(_S[0]);

                r.setRoute(path);
                r.setCost(cost);
                r.addTime(time);
                addRoute(r);
                return cost;
            }
        }

        CCdatagroup dat;
        // Initialize a CCdatagroup
        CCutil_init_datagroup(&dat);
        // UPDATE 05.08.2022
        //Convert a matrix of edge lengths to a CCdatagroup
        rval = CCutil_graph2dat_matrix(_S.size(), dim, elist, elen, 1, &dat);
        // UPDATE 04.08.2022
//        int rval = CCutil_graph2dat_matrix(_S.size(), ecount, elist, elen, 1, &dat);
//        (*G->MVIRP_logfile) << "rval = " << rval << endl;
        CCrandstate rstate;
        int seed = (int) CCutil_real_zeit();
//        int seed = rand();
        CCutil_sprand(seed, &rstate);
        // UPDATE 04.08.2022
        int *in_tour = (int *) NULL;
        int *out_tour = new int[_S.size()];
//        char *name = (char *) NULL;
//        name = CCtsp_problabel("_");
        double optval = 0;
        // UPDATE 05.08.2022
        int success, found_tour, hit_timebound = 0;
//        cout << "HERE 0" << endl;
        //Solves the TSP over the graph specified in the datagroup
        rval = CCtsp_solve_dat(_S.size(), &dat, in_tour, out_tour, NULL, &optval, &success, &found_tour, NULL, NULL, &hit_timebound, 1, &rstate);
//        int *in_tour = (int *) NULL;
//        int *out_tour = (int *) NULL;
//        CCtsp_solve_dat(_S.size(), &dat, in_tour, out_tour, NULL, &optval, &success, &found_tour, NULL, NULL, &hit_timebound, 1, &rstate);
//        cout << "HERE 1" << endl;
        for (int i = 0; i < _S.size(); i++) {
            path.push_back(_S[out_tour[i]]);
        }
        path.push_back(_S[0]);
        cost = optval;

        delete []out_tour;
        delete []elist;
        delete []elen;
        CCutil_freedatagroup(&dat);

        r.setRoute(path);
        r.setCost(cost);
        r.addTime(time);
        addRoute(r);
        return cost;
    }

    string Routes_Generator::createVRP_file(int &time, list<std::tuple<int, Vertex, float> >& data) {
        MapVertexCoordX Vertex_X = get(vertex_coordx_t(), *(G->graph));
        MapVertexCoordY Vertex_Y = get(vertex_coordy_t(), *(G->graph));
        vertex_range_t Vp = vertices(*G->graph);
        stringstream ss;
        int dimension = data.size();
        ss << G->Instance_name << "_VRP_t" << to_string(time) << "_id" << counter;
        string VRP_name = ss.str();
        ss.str("");
        ss.clear();
        ss << G->VRP_folder << "/VRP_t" << to_string(time) << "_id" << counter << ".vrp";
        string path = ss.str();
        ofstream file(path);
        file << "NAME:    " << VRP_name << endl;
        file << "DIMENSION:   " << dimension << endl;
        file << "CAPACITY:    " << G->Q << endl;
        file << "EDGE_WEIGHT_TYPE: EUC_2D" << endl;
        file << "NODE_COORD_SECTION" << endl;
        for (const auto & p : data) {
            file << get<0>(p) << "   " << Vertex_X[get<1>(p)] << "    " << Vertex_Y[get<1>(p)] << endl;
        }
        file << "DEMAND_SECTION" << endl;
        for (auto & p : data) {
            file << get<0>(p) << "   " << get<2>(p) << endl;
        }
        file << "DEPOT_SECTION" << endl;
        file << 1 << endl;
        file << -1 << endl;
        file << "EOF" << endl;
        file.close();
        return path;
    }

    void Routes_Generator::solve_VRP(int& time, list<std::tuple<int, Vertex, float> >& data) {
        string path = createVRP_file(time, data);
        float solution_val1 = 0;
        float solution_val2 = 0;
        float best_cost = 0;
        int *sol_buff = new int [data.size() + 1];
        VRP vrp1(data.size() - 1);
        VRP vrp2(data.size() - 1);
        vrp1.read_TSPLIB_file(path.c_str());
        vrp2.read_TSPLIB_file(path.c_str());
        vrp1.create_default_routes();
        ClarkeWright alg1(data.size() - 1);
        bool res1 = alg1.Construct(&vrp1, .5 + lcgrand(1), false);
        vrp1.RTR_solve(ONE_POINT_MOVE + TWO_POINT_MOVE + TWO_OPT + THREE_OPT,
                30, 5, 2, .01, 30, VRPH_LI_PERTURB, VRPH_BEST_ACCEPT, false);
        solution_val1 = vrp1.get_best_total_route_length();
        
        float total_demand = 0;
        for (auto & p : data) {
            total_demand += get<2>(p);
        }
        
//        if (data.size() >= G->num_vehicles) {
        if (ceil((float) (total_demand / G->Q)) > 1) {
            Sweep alg2;
            alg2.Construct(&vrp2);
            vrp2.RTR_solve(ONE_POINT_MOVE + TWO_POINT_MOVE + TWO_OPT + THREE_OPT,
                    30, 5, 1, .01, 25, VRPH_LI_PERTURB, VRPH_BEST_ACCEPT, false);

            solution_val2 = vrp2.get_best_total_route_length();
            if (solution_val1 >= solution_val2) {
                best_cost = solution_val2;
                vrp2.export_canonical_solution_buff(sol_buff);
            } else {
                best_cost = solution_val1;
                vrp1.export_canonical_solution_buff(sol_buff);
            }
        } else {
            best_cost = solution_val1;
            vrp1.export_canonical_solution_buff(sol_buff);
        }
        //        for(int i = 0; i < data.size() + 1; ++i) {
        //            if (sol_buff[i] < 0) {
        //                (*G->MVIRP_logfile) << "sol_buff[" << i << "] = " << sol_buff[i] - 1 << " - ";
        //            } else {
        //                (*G->MVIRP_logfile) << "sol_buff[" << i << "] = " << sol_buff[i] + 1 << " - ";
        //            }
        //        }
        //        (*G->MVIRP_logfile) << endl;
        //        list<list<Vertex> > routes; 
        bool stop = false;
        list<Vertex> vrp_route = {vertex(0, *(G->graph))};
        auto find_tuple = find_if(data.begin(), data.end(), [&](const std::tuple<int, Vertex, float>& t) {
            return get<0>(t) == -(sol_buff[1] - 1);
        });
        if (find_tuple != data.end())
            vrp_route.emplace_back(get<1>(*find_tuple));

        for (int i = 2; i <= data.size() && !stop; ++i) {
            if (sol_buff[i] <= 0) {
                if (use_CONCORDE) {
                    applyTSP(time, vrp_route);
                } else {
                    vrp_route.emplace_back(vertex(0, *(G->graph)));
                    Route r = Route();
                    r.setRoute(vrp_route);
                    r.computeCost(G->graph);
                    r.addTime(time);
                    addRoute(r);
                }

                vrp_route.clear();
                if (sol_buff[i] == 0)
                    stop = true;
                else {
                    vrp_route.emplace_back(vertex(0, *(G->graph)));
                    find_tuple = find_if(data.begin(), data.end(), [&](const std::tuple<int, Vertex, float>& t) {
                        return get<0>(t) == -(sol_buff[i] - 1);
                    });
                    if (find_tuple != data.end())
                        vrp_route.emplace_back(get<1>(*find_tuple));
                }
            } else {
                find_tuple = find_if(data.begin(), data.end(), [&](const std::tuple<int, Vertex, float>& t) {
                    return get<0>(t) == (sol_buff[i] + 1);
                });
                if (find_tuple != data.end())
                    vrp_route.emplace_back(get<1>(*find_tuple));
            }
        }
        delete sol_buff;
    }

    void Routes_Generator::get_VRP_solutions(
    vector<vector<list<std::tuple<int, Vertex, float> > > >& p_maps) {
        clock_t c_start = clock();
        for (auto& zq_sol : p_maps) {
            for (int t = 1; t <= G->H; ++t) {
                if (!zq_sol[t - 1].empty()) {
                    solve_VRP(t, zq_sol[t - 1]);
                }
            }
            counter++;
        }
        clock_t c_end = clock();
        long double time_elapsed_routes_generation = (c_end - c_start) / (double) CLOCKS_PER_SEC;
        (*G->MVIRP_logfile) << "COMPUTING TIME TO BUILD ROUTES (sec.)" << ": " << time_elapsed_routes_generation << endl;
    }

}


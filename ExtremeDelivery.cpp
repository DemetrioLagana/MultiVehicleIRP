/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Quantities_model.cpp
 * Author: demetrio
 * 
 * Created on 9 aprile 2022, 10.01
 */

#include "ExtremeDelivery.hpp"

namespace mvirp {

    ExtremeDelivery_model::ExtremeDelivery_model(MVIRP_graph &Graph) {
        counter = 1;
        G = &Graph;
        export_model = G->cfg->getValueOfKey<bool>("EXPORT_MODEL");
        flipping_threshold = G->cfg->getValueOfKey<int>("FLIPPING_THRESHOLD");
        num_times_solve_model = G->cfg->getValueOfKey<int>("NUM_TIMES_SOLVE_MODEL");
        MIP_emphasis = G->cfg->getValueOfKey<int>("MIP_EMP");
        probing = G->cfg->getValueOfKey<int>("PROBING");
        time = G->cfg->getValueOfKey<int>("TIME_LIMIT");
        var_sel = G->cfg->getValueOfKey<int>("VAR_SEL");
        tabu_tenure = G->cfg->getValueOfKey<int>("TABU_TENURE");

        env = IloEnv();
        model = IloModel(env);
        cplex = IloCplex(model);

        q = THREE_INDEX_VAR(env, G->num_vertices - 1);
        y = THREE_INDEX_VAR(env, G->num_vertices - 1);
        z = TWO_INDEX_VAR(env, G->num_vertices - 1);

        rng = RANGE_CONSTRAINTS(env);
        flrng = RANGE_FLIPPING_CONSTRAINTS(env);
    }

    ExtremeDelivery_model::~ExtremeDelivery_model() {
        rng.clear();
        rng.endElements();
        rng.end();
        cplex.clear();
        cplex.end();
        model.end();
        env.end();
    }

    void ExtremeDelivery_model::define_variables() {
        stringstream name;
        string s_name;
        MapVertexIndex VertexIndex = get(vertex_index, *(G->graph));
        MapVertexU VertexU = get(vertex_demandupperbound_t(), *(G->graph));
        MapVertexTplus VertexTplus = get(vertex_Tplus_t(), *(G->graph));

        vertex_range_t Vp = vertices(*(G->graph));
        Vp.first++;
        try {
            for (; Vp.first != Vp.second; ++Vp.first) {
                q[VertexIndex[*Vp.first] - 1] = TWO_INDEX_VAR(env, G->H);
                y[VertexIndex[*Vp.first] - 1] = TWO_INDEX_VAR(env, G->H);
                z[VertexIndex[*Vp.first] - 1] = ARRAY_VAR(env, G->H);
                for (int p = 1; p <= G->H; ++p) {
                    q[VertexIndex[*Vp.first] - 1][p - 1] = ARRAY_VAR(env, VertexTplus[*Vp.first][p - 1].size());
                    y[VertexIndex[*Vp.first] - 1][p - 1] = ARRAY_VAR(env, VertexTplus[*Vp.first][p - 1].size());
                    for (int s = 0; s < VertexTplus[*Vp.first][p - 1].size(); ++s) {
                        if (VertexTplus[*Vp.first][p - 1][s] <= G->H) {
                            name.str("");
                            name.clear();
                            name << "q[" << VertexIndex[*Vp.first] + 1 << "][" << p << "][" << VertexTplus[*Vp.first][p - 1][s] << "]";
                            s_name = name.str();
                            q[VertexIndex[*Vp.first] - 1][p - 1][s] = IloNumVar(env, 0, VertexU[*Vp.first][p - 1][s], ILOFLOAT);
                            q[VertexIndex[*Vp.first] - 1][p - 1][s].setName(&(s_name[0]));

                            name.str("");
                            name.clear();
                            name << "y[" << VertexIndex[*Vp.first] + 1 << "][" << p << "][" << VertexTplus[*Vp.first][p - 1][s] << "]";
                            s_name = name.str();
                            y[VertexIndex[*Vp.first] - 1][p - 1][s] = IloNumVar(env, 0, 1, ILOBOOL);
                            y[VertexIndex[*Vp.first] - 1][p - 1][s].setName(&(s_name[0]));
                        }
                    }

                    //                    q[VertexIndex[*Vp.first] - 1][p - 1] = ARRAY_VAR(env, G->H);
                    //                    y[VertexIndex[*Vp.first] - 1][p - 1] = ARRAY_VAR(env, G->H);
                    //                    for (int s = 1; s <= G->H; ++s) {
                    //                        auto it = find(VertexTplus[*Vp.first][p - 1].begin(), VertexTplus[*Vp.first][p - 1].end(), s);
                    //                        name.str("");
                    //                        name.clear();
                    //                        name << "q[" << VertexIndex[*Vp.first] + 1 << "][" << p << "][" << s << "]";
                    //                        s_name = name.str();
                    //                        if (it != VertexTplus[*Vp.first][p - 1].end()) {
                    //                            int index = it - VertexTplus[*Vp.first][p - 1].begin();
                    //                            q[VertexIndex[*Vp.first] - 1][p - 1][s - 1] = IloNumVar(env, 0, VertexU[*Vp.first][p - 1][index], ILOFLOAT);
                    //                            y[VertexIndex[*Vp.first] - 1][p - 1][s - 1] = IloNumVar(env, 0, 1, ILOBOOL);
                    //                        } else {
                    //                            q[VertexIndex[*Vp.first] - 1][p - 1][s - 1] = IloNumVar(env, 0, 0, ILOFLOAT);
                    //                            y[VertexIndex[*Vp.first] - 1][p - 1][s - 1] = IloNumVar(env, 0, 0, ILOBOOL);
                    //                        }
                    //                        q[VertexIndex[*Vp.first] - 1][p - 1][s - 1].setName(&(s_name[0]));
                    //                        
                    //                        name.str("");
                    //                        name.clear();
                    //                        name << "y[" << VertexIndex[*Vp.first] + 1 << "][" << p << "][" << s << "]";
                    //                        s_name = name.str();
                    //                        y[VertexIndex[*Vp.first] - 1][p - 1][s - 1].setName(&(s_name[0]));
                    //                    }
                    name.str("");
                    name.clear();
                    name << "z[" << VertexIndex[*Vp.first] + 1 << "][" << p << "]";
                    s_name = name.str();
                    z[VertexIndex[*Vp.first] - 1][p - 1] = IloNumVar(env, 0, 1, ILOBOOL);
                    z[VertexIndex[*Vp.first] - 1][p - 1].setName(&(s_name[0]));
                }
            }

        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "define_variables(): Concert exception caught: " << e << endl;
        } catch (...) {
            (*G->MVIRP_logfile) << "define_variables(): Unknown exception caught" << endl;
        }
    }

    void ExtremeDelivery_model::define_constraints() {
        stringstream ss;
        MapVertexIndex VertexIndex = get(vertex_index, *(G->graph));
        MapVertexTplus VertexTplus = get(vertex_Tplus_t(), *(G->graph));
        MapVertexU VertexU = get(vertex_demandupperbound_t(), *(G->graph));
        MapVertexTminus VertexTminus = get(vertex_Tminus_t(), *(G->graph));
        IloRange constraint;

        vertex_range_t Vp = vertices(*(G->graph));
        Vp.first++;
        try {
            for (int p = 1; p <= G->H; ++p) {
                IloExpr AggCap(env);
                ss.str("");
                ss.clear();
                ss << "AggCap_p:" << p;
                set<int> periods = G->get_T_sets()[p - 1];
                for (auto& id_vi : periods) {
                    //                    cout << id_vi << endl;
                    Vertex v = vertex(id_vi, *(G->graph));
                    //                    cout << "Vertex = " << VertexIndex[v] << endl;
                    //                    cout << "VertexTplus[v][p - 1].size() = " << VertexTplus[v][p - 1].size() << endl;
                    for (int s = 0; s < VertexTplus[v][p - 1].size(); ++s) {
                        if (VertexTplus[v][p - 1][s] <= G->H) {
                            //                            cout << "id_vi - 1 = " << id_vi - 1 << " - p - 1 = " << p - 1 << " - s = " << s << endl;
                            //                            cout << q_map[id_vi - 1][p - 1][s].getName() << endl;
                            AggCap += q[id_vi - 1][p - 1][s];
                        }
                    }
                }
                model.add(AggCap <= G->num_vehicles * G->Q).setName(&(ss.str()[0]));
                constraint = (AggCap <= G->num_vehicles * G->Q);
                constraint.setName(&(ss.str()[0]));
                rng.add(constraint);

                AggCap.end();
            }
            // ADDED IN JULY 27 2022 TO ACCOUNT FOR THE FACT THAT SUM_{s \in T^+_{ip}} q^s_{ip} <= Q, WHERE
            // Q IS THE VEHICLE CAPACITY. THESE CONSTRAINTS IMPOSE THAT THE TOTAL AMOUNT DELIVERED TO
            // CUSTOMER i IN PERIOD p TO COVER THE DEMANDS FROM p TO ANY SUCCESSIVE PERIOD CANNOT BE
            // GREATER THAN THE VEHICLE CAPACITY
            for (; Vp.first != Vp.second; ++Vp.first) {
                for (int p = 1; p <= G->H; ++p) {
                    IloExpr UpperDeliverySingleCustomer(env);
                    ss.str("");
                    ss.clear();
                    ss << "UpperDeliverySingleCustomer_v:" << VertexIndex[*Vp.first] + 1 << "p:" << p;
                    for (int s = 0; s < VertexTplus[*Vp.first][p - 1].size(); ++s) {
                        if (VertexTplus[*Vp.first][p - 1][s] <= G->H) {
                            UpperDeliverySingleCustomer += q[VertexIndex[*Vp.first] - 1][p - 1][s];
                        }
                    }
                    model.add(UpperDeliverySingleCustomer <= G->Q).setName(&(ss.str()[0]));
                    constraint = (UpperDeliverySingleCustomer <= G->Q);
                    constraint.setName(&(ss.str()[0]));
                    rng.add(constraint);

                    UpperDeliverySingleCustomer.end();
                }
            }
            
            Vp = vertices(*(G->graph));
            Vp.first++;
            for (; Vp.first != Vp.second; ++Vp.first) {
                for (int p = 1; p <= G->H; ++p) {
                    for (int s = 0; s < VertexTplus[*Vp.first][p - 1].size(); ++s) {
                        if (VertexTplus[*Vp.first][p - 1][s] <= G->H) {
                            IloExpr UpperDelivery(env);
                            ss.str("");
                            ss.clear();
                            ss << "UpperDelivery_v:" << VertexIndex[*Vp.first] + 1 << "p:" << p << "s:" << s;
                            UpperDelivery += q[VertexIndex[*Vp.first] - 1][p - 1][s];
                            UpperDelivery -= VertexU[*Vp.first][p - 1][s] * y[VertexIndex[*Vp.first] - 1][p - 1][s];

                            model.add(UpperDelivery <= 0).setName(&(ss.str()[0]));
                            constraint = (UpperDelivery <= 0);
                            constraint.setName(&(ss.str()[0]));
                            rng.add(constraint);

                            UpperDelivery.end();

                            IloExpr YZ(env);
                            ss.str("");
                            ss.clear();
                            ss << "YZ_v:" << VertexIndex[*Vp.first] + 1 << "p:" << p << "s:" << s;
                            YZ += y[VertexIndex[*Vp.first] - 1][p - 1][s];
                            YZ -= z[VertexIndex[*Vp.first] - 1][p - 1];

                            model.add(YZ <= 0).setName(&(ss.str()[0]));
                            constraint = (YZ <= 0);
                            constraint.setName(&(ss.str()[0]));
                            rng.add(constraint);

                            YZ.end();
                        }
                    }

                    IloExpr Delivery(env);
                    ss.str("");
                    ss.clear();
                    ss << "Delivery_v:" << VertexIndex[*Vp.first] + 1 << "p:" << p;
                    for (auto& s : VertexTminus[*Vp.first][p - 1]) {
                        auto it = find(VertexTplus[*Vp.first][s - 1].begin(), VertexTplus[*Vp.first][s - 1].end(), p);
                        if (it != VertexTplus[*Vp.first][s - 1].end()) {
                            int index = it - VertexTplus[*Vp.first][s - 1].begin();
                            Delivery += y[VertexIndex[*Vp.first] - 1][s - 1][index];
                        }
                    }
                    if (!VertexTminus[*Vp.first][p - 1].empty()) {
                        model.add(Delivery <= 1).setName(&(ss.str()[0]));
                        constraint = (Delivery <= 1);
                        constraint.setName(&(ss.str()[0]));
                        rng.add(constraint);
                    }
                    Delivery.end();

                    IloExpr Zineq(env);
                    ss.str("");
                    ss.clear();
                    ss << "Zineq_v:" << VertexIndex[*Vp.first] + 1 << "p:" << p;
                    for (auto& s : VertexTminus[*Vp.first][p - 1]) {
                        Zineq += z[VertexIndex[*Vp.first] - 1][s - 1];
                    }
                    if (!VertexTminus[*Vp.first][p - 1].empty()) {
                        model.add(Zineq >= 1).setName(&(ss.str()[0]));
                        constraint = (Zineq >= 1);
                        constraint.setName(&(ss.str()[0]));
                        rng.add(constraint);
                    }
                    Zineq.end();
                }
            }
            // ADD FLIPPING CONSTRAINTS
            if (id_model > 1) {
                int constr_counter = 1;
//                if (flrng.getSize() > 0) {
//                    cout << "flrng.getSize() = " << flrng.getSize() << endl;
//                    for (int i = 0; i < flrng.getSize(); ++i) {
//                        cout << flrng[i].getName() << endl;
//                        model.remove(flrng[i]);
//                    }
//                    flrng.clear();
//                }
                for (auto& p : zq_solutions) {
                    if (p.first <= tabu_tenure) {
                        IloExpr Flipping(env);
                        ss.str("");
                        ss.clear();
                        ss << "Flipping_(id_constr_tt):" << constr_counter++ << "_" << p.first;
                        int number_of_visiting = 0;
                        for (const auto& t : p.second) {
                            if (get<2>(t) == 0) {
                                Flipping += z[get<0>(t) - 1][get<1>(t) - 1];
                            } else {
                                number_of_visiting++;
                                Flipping -= z[get<0>(t) - 1][get<1>(t) - 1];
                            }
                        }
                        model.add(Flipping >= flipping_threshold - number_of_visiting).setName(&(ss.str()[0]));
                        constraint = (Flipping >= flipping_threshold - number_of_visiting);
                        constraint.setName(&(ss.str()[0]));
                        flrng.add(constraint);

                        Flipping.end();
                    }
                }
                //                IloExpr Flipping(env);
                //                ss.str("");
                //                ss.clear();
                //                ss << "Flipping_id:" << id_model;
                //                int number_of_visiting = 0;
                //                for (const auto& t : z_q_sol) {
                //                    if (get<2>(t) == 0) {
                //                        Flipping += z[get<0>(t) - 1][get<1>(t) - 1];
                //                    } else {
                //                        number_of_visiting++;
                //                        Flipping -= z[get<0>(t) - 1][get<1>(t) - 1];
                //                    }
                //                }
                //                model.add(Flipping >= 1 - number_of_visiting).setName(&(ss.str()[0]));
                //                constraint = (Flipping >= 1 - number_of_visiting);
                //                constraint.setName(&(ss.str()[0]));
                //                flrng.add(constraint);
                //                (*G->MVIRP_logfile) << flrng << endl;
                //                assert(false);
                //                Flipping.end();
                //                z_q_sol.clear();
                //                for (int i = 0; i < flrng.getSize(); ++i) {
                //                    IloExpr e = flrng[i].getExpr();
                //                    IloExpr::LinearIterator it = e.getLinearIterator();
                //                    (*G->MVIRP_logfile) << flrng[i] << endl;
                //                }
            }

        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "define_constraints(): Concert exception caught: " << e << endl;
        } catch (...) {
            (*G->MVIRP_logfile) << "define_constraints(): Unknown exception caught" << endl;
        }
    }

    void ExtremeDelivery_model::define_objective() {
        MapVertexIndex VertexIndex = get(vertex_index, *(G->graph));
        MapVertexTplus VertexTplus = get(vertex_Tplus_t(), *(G->graph));
        MapVertexU VertexU = get(vertex_demandupperbound_t(), *(G->graph));
        stringstream ss;
        ss << "Maximize deliveries:";
        try {
            IloExpr objective(env);
            for (int p = 1; p <= G->H; ++p) {
                set<int> periods = G->get_T_sets()[p - 1];
                for (auto& id_vi : periods) {
                    Vertex v = vertex(id_vi, *(G->graph));
                    for (int s = 0; s < VertexTplus[v][p - 1].size(); ++s) {
                        if (VertexTplus[v][p - 1][s] <= G->H) {
                            objective += q[id_vi - 1][p - 1][s];
                        }
                    }

                }
                vertex_range_t Vp = vertices(*(G->graph));
                Vp.first++;
                for (; Vp.first != Vp.second; ++Vp.first) {
                    int max_u = *max_element(VertexU[*Vp.first][p - 1].begin(), VertexU[*Vp.first][p - 1].end());
                    objective -= max_u * z[VertexIndex[*Vp.first] - 1][p - 1];
                }
            }
            model.add(IloMaximize(env, objective));

        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "define_objective(): Concert exception caught: " << e << endl;
        } catch (...) {
            (*G->MVIRP_logfile) << "define_objective(): Unknown exception caught" << endl;
        }
    }

    void ExtremeDelivery_model::set_parameters_solver() {
        if (time > 0) {
//            (*G->MVIRP_logfile) << "TiLim = " << time << endl;
            cplex.setParam(IloCplex::TiLim, time);
        }
        if (MIP_emphasis > -1) {
//            (*G->MVIRP_logfile) << "MIPEmphasis = " << MIP_emphasis << endl;
            cplex.setParam(IloCplex::MIPEmphasis, MIP_emphasis);
        }
        if (var_sel > -1) {
//            (*G->MVIRP_logfile) << "VarSel = " << var_sel << endl;
            cplex.setParam(IloCplex::VarSel, var_sel);
        }
        if (probing > -1) {
//            (*G->MVIRP_logfile) << "Probing = " << probing << endl;
            cplex.setParam(IloCplex::Probe, probing);
        }
    }

    IloNum const ExtremeDelivery_model::build_model() {
        try {
            stringstream ss;
            ss.str("");
            ss << G->Problem_name << "_" << id_model << ".lp";
            LP = ss.str();

            IloNum const start_time = cplex.getCplexTime();
            define_variables();
            define_constraints();
            define_objective();
            IloNum const end_time = cplex.getCplexTime() - start_time;
            return end_time;
        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "build_model(): Concert exception caught: " << e << endl;
        } catch (...) {
            (*G->MVIRP_logfile) << "build_model(): Unknown exception caught" << endl;
        }
    }

    void ExtremeDelivery_model::solve_iteratively() {
        try {
            int iterations = 1;
            IloNum const start_time = cplex.getCplexTime();
            while (solve() && iterations < num_times_solve_model) {
//                (*G->MVIRP_logfile) << "Iteration n. = " << iterations << endl;
                iterations++;
                clear_and_restart_the_model();
            }
            IloNum const end_time = cplex.getCplexTime() - start_time;
            (*G->MVIRP_logfile) << "SETTING: NUM_TIMES_SOLVE_MODEL = " << num_times_solve_model <<
                    " - FLIPPING_THRESHOLD = " << flipping_threshold << " - TABU_TENURE = " << tabu_tenure << endl;
            (*G->MVIRP_logfile) << "COMPUTING TIME (sec.) TO SOLVE THE EXTREME DELIVERY MODEL " << num_times_solve_model << " TIMES, ITERATIVELY: " << end_time << endl;
        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "solve_iteratively(): Concert exception caught: " << e << endl;
        }
    }

    bool ExtremeDelivery_model::solve() {
        try {
            id_model = counter++;
            if (id_model > 1) {
                cplex.clear();
                cplex.extract(model);
            }
//            cout << "Here build model() before" << endl;
            IloNum const time_to_build = build_model();
//            cout << "Here build model() after" << endl;
            if (export_model) {
                cplex.exportModel(LP.c_str());
            }
            set_parameters_solver();

            IloNum const start = cplex.getCplexTime();
            cplex.solve();
            IloNum const cpxTime = cplex.getCplexTime() - start;

            if ((cplex.getStatus() == IloAlgorithm::Infeasible) || (cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded) ||
                    (cplex.getStatus() == IloAlgorithm::Unbounded)) {
                (*G->MVIRP_logfile) << "No solution found - CPLEX status = " << cplex.getCplexStatus() << endl;
                return false;
            } else if (cplex.getStatus() == IloAlgorithm::Unknown) {
//                (*G->MVIRP_logfile) << "No solution found - CPLEX status = " << cplex.getCplexStatus() << endl;
//                (*G->MVIRP_logfile) << fixed << setprecision(3) << "Best Objective value = " << cplex.getBestObjValue() << endl;
//                (*G->MVIRP_logfile) << "Number of nodes processed in the active branch-and-cut search = " << cplex.getNnodes() << endl;
//                (*G->MVIRP_logfile) << "Number of nodes remaining to be processed, or, equivalently, the number of active nodes in the tree = " << cplex.getNnodesLeft() << endl;
//                (*G->MVIRP_logfile) << "getTimetoBuildModel = " << time_to_build << " sec." << endl;
//                (*G->MVIRP_logfile) << "getCplexTime = " << cpxTime << " sec." << endl;
                (*G->MVIRP_logfile) << "No solution found - CPLEX status = " << cplex.getCplexStatus() << endl;
                return false;
            } else {
//                (*G->MVIRP_logfile) << fixed << setprecision(3) << "Objective value = " << cplex.getObjValue() << endl;
//                (*G->MVIRP_logfile) << fixed << setprecision(3) << "Best Objective value = " << cplex.getBestObjValue() << endl;
//                (*G->MVIRP_logfile) << fixed << setprecision(5) << "GAP = " << cplex.getMIPRelativeGap() << endl;
//                (*G->MVIRP_logfile) << "Number of nodes processed in the active branch-and-cut search = " << cplex.getNnodes() << endl;
//                (*G->MVIRP_logfile) << "Number of nodes remaining to be processed, or, equivalently, the number of active nodes in the tree = " << cplex.getNnodesLeft() << endl;
//                (*G->MVIRP_logfile) << "status = " << cplex.getStatus() << endl;
//                (*G->MVIRP_logfile) << "time limit = " << time << " sec." << endl;
//                (*G->MVIRP_logfile) << "getTimetoBuildModel = " << time_to_build << " sec." << endl;
//                (*G->MVIRP_logfile) << "getCplexTime = " << cpxTime << " sec." << endl;
//                bool new_solution_found = get_z_q_sol();
                if (get_z_q_sol()) {
//                    cout << "HERE TRUE new_solution_found" << endl;
//                    print_solution();
                    return true;
                } else {
//                    cout << "HERE FALSE new_solution_found" << endl;
//                    assert(false);
                    return false;
                }
            }

        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "build_and_solve(): Concert exception caught: " << e << endl;
        } catch (...) {
            (*G->MVIRP_logfile) << "build_and_solve(): Unknown exception caught" << endl;
        }
    }

    bool ExtremeDelivery_model::get_z_q_sol() {
        MapVertexIndex VertexIndex = get(vertex_index, *(G->graph));
        MapVertexTplus VertexTplus = get(vertex_Tplus_t(), *(G->graph));
        vertex_range_t Vp = vertices(*(G->graph));
        zq_sol zq;
        Vp.first++;
        try {
            for (; Vp.first != Vp.second; ++Vp.first) {
                for (int p = 1; p <= G->H; ++p) {
                    float delivery = 0;
                    for (int s = 0; s < VertexTplus[*Vp.first][p - 1].size(); ++s) {
                        if (VertexTplus[*Vp.first][p - 1][s] <= G->H) {
                            delivery += cplex.getValue(q[VertexIndex[*Vp.first] - 1][p - 1][s]);
                        }
                    }
//                    z_q_sol.emplace_back(make_tuple(VertexIndex[*Vp.first], p,
//                            IloRound(cplex.getValue(z[VertexIndex[*Vp.first] - 1][p - 1])),
//                            delivery));
                    zq.emplace_back(make_tuple(VertexIndex[*Vp.first], p,
                            IloRound(cplex.getValue(z[VertexIndex[*Vp.first] - 1][p - 1])),
                            delivery));
                }
            }
//            bool updated_zq_solutions = update_zq_solutions(zq);
//            return updated_zq_solutions;
//            cout << "get_result = " << get_result << endl;
            return update_zq_solutions(zq);
        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "get_z_q_sol(): Concert exception caught: " << e << endl;
        } catch (...) {
            (*G->MVIRP_logfile) << "get_z_q_sol(): Unknown exception caught" << endl;
        }
    }

    bool ExtremeDelivery_model::find_zq_sol(zq_sol& first, zq_sol& second) {
        if (first.size() != second.size()) {
            return false;
        } else {
            bool check = true;
            zq_sol::iterator it1, it2;
            it2 = second.begin();
            while (it2 != second.end()) {
                check = false;
                for (it1 = first.begin(); it1 != first.end(); ++it1) {
                    if (get<0>(*it1) == get<0>(*it2) && get<1>(*it1) == get<1>(*it2) &&
                            get<2>(*it1) == get<2>(*it2)) {
                        check = true;
                        break;
                    }
                }
                if (check)
                    it2++;
                else
                    return check;
            }
            return check;
        }
    }

    bool ExtremeDelivery_model::zq_sol_is_in_zq_solutions(zq_sol& zq) {
        for (auto& p : zq_solutions) {
            if (find_zq_sol(p.second, zq)) {
                return true;
            }
        }
        return false;
    }

    bool ExtremeDelivery_model::update_zq_solutions(zq_sol& zq) {
//        cout << "Here before zq_solutions.erase()" << endl;
//        list<pair<int, zq_sol> >::iterator it = zq_solutions.begin();
//        while (it != zq_solutions.end()) {
//            if (it->first + 1 > tabu_tenure) {
//                it = zq_solutions.erase(it);
//            }
//            it++;
//        }
//        cout << "Here after zq_solutions.erase()" << endl;
        for (auto& p : zq_solutions) {
            p.first += 1;
        }
        if (!zq_sol_is_in_zq_solutions(zq)) {
            zq_solutions.emplace_back(make_pair(1, zq));
            return true;
        } else {
            return false;
        }
    }

    void ExtremeDelivery_model::clear_and_restart_the_model() {
        try {
            cplex.clearModel();
            model.end();

            model = IloModel(env);
            q = THREE_INDEX_VAR(env, G->num_vertices - 1);
            y = THREE_INDEX_VAR(env, G->num_vertices - 1);
            z = TWO_INDEX_VAR(env, G->num_vertices - 1);

            rng = RANGE_CONSTRAINTS(env);
            flrng = RANGE_CONSTRAINTS(env);
        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "reset_model(): Concert exception caught: " << e << endl;
        }
    }

    list<std::tuple<int, Vertex, float> > ExtremeDelivery_model::get_p_map(int &p, zq_sol& z_q_sol) {
        int id = 1;
        list<std::tuple<int, Vertex, float> > map({make_tuple(id++, vertex(0, *(G->graph)), 0)});
        for (const auto& it : z_q_sol) {
            if (get<1>(it) == p && get<2>(it) > 0) {
                Vertex v = vertex(get<0>(it), *(G->graph));
                map.emplace_back(make_tuple(id++, v, get<3>(it)));
            }
        }
        if (map.size() == 1) {
            map.clear();
        }
        return map;
    }

    vector<vector<list<std::tuple<int, Vertex, float> > > > ExtremeDelivery_model::get_period_maps() {
        vector<vector<list<std::tuple<int, Vertex, float> > > > maps_solutions;
        for (auto& p : zq_solutions) {
            vector<list<std::tuple<int, Vertex, float> > > maps;
            for (int t = 1; t <= G->H; ++t) {
                maps.emplace_back(get_p_map(t, p.second));
            }
            maps_solutions.emplace_back(maps);
        }
        //        MapVertexIndex VertexIndex = get(vertex_index, *(G->graph));
        //        for (int p = 1; p <= G->H; ++p) {
        //            if (!maps[p - 1].empty()) {
        //                (*G->MVIRP_logfile) << "List at time " << p << ": {";
        //                for (list<std::tuple<int, Vertex, float> >::iterator it = maps[p - 1].begin();
        //                        it != maps[p - 1].end(); ++it){
        //                   if (*it != *(maps[p - 1].rbegin())) {
        //                       (*G->MVIRP_logfile) << "(" << get<0>(*it) << "," << VertexIndex[get<1>(*it)] + 1 <<
        //                               "," << get<2>(*it) << "); ";
        //                   } else {
        //                      (*G->MVIRP_logfile) << "(" << get<0>(*it) << "," << VertexIndex[get<1>(*it)] + 1 <<
        //                               "," << get<2>(*it) << ")}" << endl; 
        //                   }
        //                }
        //            }
        //        }
        return maps_solutions;
    }

    void ExtremeDelivery_model::print_solution() {
        MapVertexIndex VertexIndex = get(vertex_index, *(G->graph));
        MapVertexTplus VertexTplus = get(vertex_Tplus_t(), *(G->graph));
        vertex_range_t Vp = vertices(*(G->graph));
        Vp.first++;
        try {
            (*G->MVIRP_logfile) << "SOLUTION:" << endl;
            for (; Vp.first != Vp.second; ++Vp.first) {
                for (int p = 1; p <= G->H; ++p) {
                    bool print = false;
                    if (cplex.getValue(z[VertexIndex[*Vp.first] - 1][p - 1]) > 0) {
                        (*G->MVIRP_logfile) << z[VertexIndex[*Vp.first] - 1][p - 1].getName() << " = " <<
                                IloRound(cplex.getValue(z[VertexIndex[*Vp.first] - 1][p - 1])) << endl;
                        print = true;

                    }
                    if (print) {
                        print = false;
                        (*G->MVIRP_logfile) << "{ ";
                        for (int s = 0; s < VertexTplus[*Vp.first][p - 1].size(); ++s) {
                            if (VertexTplus[*Vp.first][p - 1][s] <= G->H) {
                                if (cplex.getValue(q[VertexIndex[*Vp.first] - 1][p - 1][s]) > 0) {
                                    (*G->MVIRP_logfile) << "(" << q[VertexIndex[*Vp.first] - 1][p - 1][s].getName() << " = " <<
                                            cplex.getValue(q[VertexIndex[*Vp.first] - 1][p - 1][s]) << "," <<
                                            y[VertexIndex[*Vp.first] - 1][p - 1][s].getName() << " = " <<
                                            IloRound(cplex.getValue(y[VertexIndex[*Vp.first] - 1][p - 1][s])) << ") ";
                                    print = true;
                                }

                            }
                        }
                        (*G->MVIRP_logfile) << " }" << endl;

                        assert(print);
                    }
                }
            }

        } catch (IloException& e) {
            (*G->MVIRP_logfile) << "print_solution(): Concert exception caught: " << e << endl;
        } catch (...) {
            (*G->MVIRP_logfile) << "print_solution(): Unknown exception caught" << endl;
        }
    }
}


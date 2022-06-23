/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Definitions.hpp
 * Author: demetrio
 *
 * Created on 31 marzo 2022, 12.22
 */

#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <boost/graph/adjacency_list.hpp>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace boost;

namespace mvirp {
    
    const double eps = 0.00001;

    const int max_int = numeric_limits<int>::infinity();

    const double max_double = numeric_limits<double>::infinity();
    
    //Vertex type
    struct vertex_coordx_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_coordy_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_production_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_demand_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_inv0_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_mininv_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_maxinv_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_invcost_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_inv0remaining_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_demandremaining_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_demandupperbound_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_Tplus_t {
        typedef vertex_property_tag kind;
    };
    
    struct vertex_Tminus_t {
        typedef vertex_property_tag kind;
    };
    
    typedef property<vertex_index_t, int, no_property> VertexIndexProperty;
    typedef property<vertex_name_t, string, VertexIndexProperty> VertexNameProperty;
    typedef property<vertex_coordx_t, float, VertexNameProperty> VertexCoordXProperty;
    typedef property<vertex_coordy_t, float, VertexCoordXProperty> VertexCoordYProperty;
    typedef property<vertex_production_t, vector<int>, VertexCoordYProperty> VertexProductionProperty;
    typedef property<vertex_demand_t, vector<int>, VertexProductionProperty> VertexDemandProperty;
    typedef property<vertex_inv0_t, int, VertexDemandProperty> VertexInvInitProperty;
    typedef property<vertex_mininv_t, int, VertexInvInitProperty> VertexInvMinProperty;
    typedef property<vertex_maxinv_t, int, VertexInvMinProperty> VertexInvMaxProperty;
    typedef property<vertex_invcost_t, float, VertexInvMaxProperty> VertexInvCostProperty;
    typedef property<vertex_inv0remaining_t, vector<int>, VertexInvCostProperty> VertexInv0s;
    typedef property<vertex_demandremaining_t, vector<int>, VertexInv0s> VertexBarDs;
    typedef property<vertex_demandupperbound_t, vector<vector<int> >, VertexBarDs> VertexU;
    typedef property<vertex_Tplus_t, vector<vector<int> >, VertexU> VertexTplus;
    typedef property<vertex_Tminus_t, vector<vector<int> >, VertexTplus> VertexTminus;
    typedef property<vertex_color_t, boost::default_color_type, VertexTminus> VertexProperties;
    
    // Arc type
    typedef double EdgeWeightType;
    
    struct edge_weight_t {
        typedef edge_property_tag kind;
    };
    
    struct edge_cost_t {
        typedef edge_property_tag kind;
    };
    
    typedef property<edge_index_t, int, no_property> EdgeIndexProperty;
    typedef property<edge_weight_t, float, EdgeIndexProperty> EdgeWeightProperty;
    typedef property<edge_cost_t, float, EdgeWeightProperty> EdgeCostProperty;
    typedef property<edge_name_t, string, EdgeCostProperty> EdgeProperties;
    
    typedef adjacency_list<multisetS, vecS, undirectedS, VertexProperties, EdgeProperties> UndirectedGraph;
    
    typedef property_map<UndirectedGraph, vertex_name_t>::type MapVertexName;
    typedef property_map<UndirectedGraph, vertex_index_t>::type MapVertexIndex;
    typedef property_map<UndirectedGraph, vertex_coordx_t>::type MapVertexCoordX;
    typedef property_map<UndirectedGraph, vertex_coordy_t>::type MapVertexCoordY;
     typedef property_map<UndirectedGraph, vertex_production_t>::type MapVertexProduction;
    typedef property_map<UndirectedGraph, vertex_demand_t>::type MapVertexDemand;
    typedef property_map<UndirectedGraph, vertex_inv0_t>::type MapVertexInv0;
    typedef property_map<UndirectedGraph, vertex_mininv_t>::type MapVertexInvMin;
    typedef property_map<UndirectedGraph, vertex_maxinv_t>::type MapVertexInvMax;
    typedef property_map<UndirectedGraph, vertex_invcost_t>::type MapVertexInvCost;
    
    typedef property_map<UndirectedGraph, vertex_inv0remaining_t>::type MapVertexI0s;
    typedef property_map<UndirectedGraph, vertex_demandremaining_t>::type MapVertexBarDs; 
    typedef property_map<UndirectedGraph, vertex_demandupperbound_t>::type MapVertexU;
    typedef property_map<UndirectedGraph, vertex_Tplus_t>::type MapVertexTplus;
    typedef property_map<UndirectedGraph, vertex_Tminus_t>::type MapVertexTminus;
    typedef property_map<UndirectedGraph, vertex_color_t>::type MapVertexColor;
    
    typedef property_map<UndirectedGraph, edge_name_t>::type MapEdgeName;
    typedef property_map<UndirectedGraph, edge_index_t>::type MapEdgeIndex;
    typedef property_map<UndirectedGraph, edge_weight_t>::type MapEdgeWeight;
    typedef property_map<UndirectedGraph, edge_cost_t>::type MapEdgeCost;
    
    typedef graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
    typedef graph_traits<UndirectedGraph>::edge_descriptor Edge;
    typedef graph_traits<UndirectedGraph>::vertex_iterator vertex_iter;
    typedef graph_traits<UndirectedGraph>::edge_iterator edge_iter;
    typedef graph_traits<UndirectedGraph>::adjacency_iterator adjacency_iter;
    typedef graph_traits<UndirectedGraph>::vertices_size_type Num_Vertices;
    typedef graph_traits<UndirectedGraph>::edges_size_type Num_Edges;
    typedef pair<Edge, bool> Find_Edge;
    typedef pair<adjacency_iter, adjacency_iter> adjacency_range_t;
    typedef pair<vertex_iter, vertex_iter> vertex_range_t;
    typedef pair<edge_iter, edge_iter> edge_range_t;
    
    
    //CPLEX
    typedef IloEnv ENV;
    typedef IloModel MODEL;
    typedef IloCplex CPLEX;
    
    typedef IloNumVarArray ARRAY_VAR;
    typedef IloArray<ARRAY_VAR> TWO_INDEX_VAR;
    typedef IloArray<TWO_INDEX_VAR> THREE_INDEX_VAR;
    
    typedef IloExprArray EXPRS;
    typedef IloRangeArray RANGE_CONSTRAINTS;
    typedef IloRangeArray RANGE_FLIPPING_CONSTRAINTS;
    
}

std::string trim(const std::string& str,
        const std::string& whitespace = " \t") {
    const int strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content

    const int strEnd = str.find_last_not_of(whitespace);
    const int strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

std::string reduce(const std::string& str,
        const std::string& fill = " ",
        const std::string& whitespace = "; \t") {
    // trim first
    std::string result = trim(str, whitespace);

    // replace sub ranges
    int beginSpace = result.find_first_of(whitespace);
    while (beginSpace != std::string::npos) {
        const int endSpace = result.find_first_not_of(whitespace, beginSpace);
        const int range = endSpace - beginSpace;

        result.replace(beginSpace, range, fill);

        const int newStart = beginSpace + fill.length();
        beginSpace = result.find_first_of(whitespace, newStart);
    }

    return result;
}

std::ostream &operator<<(std::ostream &os, const std::vector<int> &input)
{
    os << "{";
    for (vector<int>::const_iterator it = input.begin(); it != input.end(); ++it) {
        if (it != input.end() - 1)
            os << *it << ", ";
        else
            os << *it;
    }
    os << "}";
//    for (auto const &i: input) {
//        os << i << " ";
//    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const std::set<int> &input)
{
    os << "{";
    for (set<int>::const_iterator it = input.begin(); it != input.end(); ++it) {
        if (*it != *(input.rbegin()))
            os << *it + 1 << ", ";
        else
            os << *it + 1;
    }
    os << "}";
//    for (auto const &i: input) {
//        os << i << " ";
//    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const std::vector<vector<int> > &input)
{
    os << "[";
    for (vector<vector<int> >::const_iterator it = input.begin(); it != input.end(); ++it) {
        if (it != input.end() - 1)
            os << *it << ", ";
        else
            os << *it;
    }
    os << "]";
//    for (auto const &i: input) {
//        os << i << " ";
//    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const std::vector<set<int> > &input)
{
    os << "[";
    for (vector<set<int> >::const_iterator it = input.begin(); it != input.end(); ++it) {
        if (it != input.end() - 1)
            os << *it << ", ";
        else
            os << *it;
    }
    os << "]";
//    for (auto const &i: input) {
//        os << i << " ";
//    }
    return os;
}

#endif /* DEFINITIONS_HPP */


//
// Created by usuario on 13/12/23.
//

#ifndef ROOMS_GRAPH_H
#define ROOMS_GRAPH_H
#include <vector>
#include <iostream>

class Graph
{
    using Nodes = std::vector<int>;
    using Edges = std::vector<std::pair<int,int>>;
    Nodes nodes;
    Edges edges;
    int current_node = 0;

public:
    Graph();
    int add_node();
    int add_edge (int n1, int n2);

    size_t get_node_count() const {
        return nodes.size();
    }


    // Método para obtener una copia del vector de nodos
    std::vector<int> getNodes() const {
        return nodes;
    }

    void print();
};


#endif //ROOMS_GRAPH_H

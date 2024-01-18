/******************************************************************************
Aviso Legal: Este código fuente está protegido por derechos de autor y es
propiedad exclusiva de Miguel Medina Cantos, estudiante de la Carrera de Ingeniería
Informática en Ingeniería de Computadores. La copia o reproducción total o parcial
de este código sin permiso escrito de Miguel Medina Cantos está estrictamente prohibida,
excepto en los siguientes casos:

1. El profesor de robótica tiene permiso para copiar y utilizar este código tal cual para cualquier
   propósito.

Cualquier usuario que copie este código está requerido a realizar cambios sustanciales
en el mismo antes de su uso en cualquier aplicación o proyecto. La mera copia sin
modificaciones sustanciales no está permitida y puede incurrir en acciones legales
por parte de Miguel Medina Cantos.

Miguel Medina Cantos
Carrera de Ingeniería Informática en Ingeniería de Computadores
11 de enero de 2024, 17:16

Este aviso legal se aplica a todos los componentes y archivos incluidos en este
proyecto, incluyendo, pero sin limitarse a, código fuente, documentación y archivos
de configuración.

© 2024 Miguel Medina Cantos. Todos los derechos reservados.
******************************************************************************/

#include "graph.h"
#include <ranges>

Graph::Graph()
{
    nodes.push_back(0);
}

int Graph::add_node()
{
    nodes.push_back(nodes.size());
    return nodes.size()-1;
}

int Graph::add_edge(int n1, int n2) {
    if (std::ranges::find(nodes, n1) != nodes.end() and
        std::ranges::find(nodes, n2) != nodes.end() and
        std::ranges::find(edges, std::make_pair(n1, n2)) == edges.end())
    {
        edges.emplace_back(n1, n2);
        return 1;
    }
     else return -1;
}


// Implementación de get_node_count
size_t Graph::get_node_count() const {
    return nodes.size();
}

// Implementación de getNodes
std::vector<int> Graph::getNodes() const {
    return nodes;
}




void Graph::print()
{

    std::cout << "Nodos grafo: "  << " ";


    for (const auto &n : nodes)
    {
        std::cout<< n << " " ;
    }

    std::cout<<std::endl;

    std::cout<< "Arcos grafo:   " <<  " ";

    for (const auto &e : edges)
    {
        std::cout<< e.first << " "  << e.second<<std::endl;
    }

    std::cout<<std::endl;

}

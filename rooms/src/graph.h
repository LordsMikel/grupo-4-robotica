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


    // Declaración del método para obtener la cantidad de nodos
    size_t get_node_count() const;

    // Declaración del método para obtener una copia del vector de nodos
    std::vector<int> getNodes() const;




    void print();
};


#endif //ROOMS_GRAPH_H

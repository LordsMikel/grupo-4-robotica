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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>
#include <tuple>
#include "door_detector.h"
#include <Eigen/Dense>
#include "graph.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
    void compute();
    int startup_check();
    void initialize(int period);

private:
    bool startup_check_flag;
    AbstractGraphicViewer *viewer;

    struct Constants
    {
        std::string lidar_name = "helios";
        const float MAX_ADV_SPEED = 700;
        const float DOOR_PROXIMITY_THRESHOLD = 200;
        std::vector<std::pair<float, float>> ranges_list = {{1000, 2000}};
    };
    Constants consts;

    using Door = DoorDetector::Door;
    using Doors = std::vector<Door>;
    using Line = std::vector<Eigen::Vector2f>;
    using Lines = std::vector<Line>;

    // Doors
    DoorDetector door_detector;
    std::vector<Line> extract_lines(const RoboCompLidar3D::TPoints &points, const vector<std::pair<float, float>> &ranges);
    void match_door_target(const Doors &doors, const Door &target);

    // Draw
    void draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);
    void draw_target_door(const Door &target, AbstractGraphicViewer *viewer, QColor color="magenta", QColor color_far="orange");
    void draw_lines(const Lines &lines, AbstractGraphicViewer *pViewer);

    // states
    Door door_target;
    enum class States{ IDLE, SEARCH_DOOR, GOTO_DOOR, GO_THROUGH, ALIGN};
    States state = States::SEARCH_DOOR;
    void state_machine(const Doors &doors);

    // robot
    void move_robot(float side, float adv, float rot);
    float break_adv(float dist_to_target);
    float break_rot(float rot);

    Graph graph;



    // Nuevo miembro para rastrear la habitación actual
    int contadorHabitacion = 0;



    // El cronometro de 10s, para las habitaciones
    std::chrono::steady_clock::time_point goThroughStartTime;

};
#endif
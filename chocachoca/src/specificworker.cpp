/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
    // Uncomment if there's too many debug messages
    // but it removes the possibility to see the messages
    // shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(460, 480, 0, 100, QColor("Blue"));
        viewer->show();
        viewer->activateWindow();
        timer.start(Period);

    }

}

void SpecificWorker::compute() {



    RoboCompLidar3D::TData ldata;


    ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
    const auto &points = ldata.points;
    if(points.empty()) return;





    RoboCompLidar3D::TPoints filtered_points;
    std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) { return p.z < 2000;});



    draw_lidar(filtered_points, viewer);
    std::tuple<Estado, RobotSpeed> res;





    switch(estado)
    {
        //Cambiar después de IDLE:
        case Estado::IDLE: {
            res = spiral();
            break;
        }

        case Estado::FOLLOW_WALL:
            res = follow_wall(filtered_points);
            break;
        case Estado::STRAIGHT_LINE: {
            res = chocachoca(filtered_points);
            break;
        }
        case Estado::TURN: {
            res = turn(filtered_points);
            break;
        }

        case Estado::SPIRAL:
            break;
    }


    //Desempaquetamos la tupla.
    estado = std::get<0>(res);
    RobotSpeed robot_speed_res = std::get<1>(res);
    const auto &[adv, side, rot] = robot_speed_res;
    omnirobot_proxy->setSpeedBase(adv, side, rot);

}


//Convertir el estado turn lo convertimos en el estado intermedio IMP ENTONCES VAMOS CAMBIANDO ENTRE ESTADOS ALETORIEDAD MIRANDO LA DISTANCIA MINIMA A LA PARED SINO VOLVEMOS AL ESTADO ANTERIOR

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::turn(RoboCompLidar3D::TPoints &points) {

    // Definir los índices de inicio y fin para el rango lateral
    int start_offset_lat = points.size() / 6;
    int end_offset_lat =  (points.size() * 2)/3;

    // Encontrar el punto más cercano en el rango lateral
    auto min_elem_lat = std::min_element(points.begin() + start_offset_lat, points.end() - end_offset_lat,
                                         [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    // Calcular el ángulo del punto más cercano
    float angle = std::atan2(min_elem_lat->y, min_elem_lat->x);



    // Definir los valores por defecto
    Estado state = Estado::FOLLOW_WALL;  // Asumiendo que volveremos a FOLLOW_WALL
    RobotSpeed robot_speed;


    const float UmbralRot = 3.0;

    float rotAngular = UmbralRot * angle;


    if(std::abs(angle) < 0.1) {  // Ajusta el valor 0.1 según sea necesario
        qInfo()<< "Cambiando";
        robot_speed = RobotSpeed{.adv=1.0, .side=0, .rot=UmbralRot};
    } else {
        qInfo()<< "Cambiando 2";

        robot_speed = RobotSpeed{.adv=0, .side=0, .rot=UmbralRot};
    }

    return std::make_tuple(state, robot_speed);
}



std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points){

    // Define los índices de inicio y fin para el rango lateral
    int start_offset_lat = points.size() / 6;
    int end_offset_lat = (points.size() * 2)/3;

    // Encuentra el punto más cercano en el rango lateral
    auto min_elem_lat = std::min_element(points.begin() + start_offset_lat, points.end() - end_offset_lat,
                                         [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    // Calcula el ángulo y la distancia al punto más cercano
    float angle = std::atan2(min_elem_lat->y, min_elem_lat->x);
    float lateral_distance = std::hypot(min_elem_lat->x, min_elem_lat->y);

    RobotSpeed robot_speed;
    Estado estado;
    const float REFERENCE_DISTANCE = 900;  // Asume una distancia de referencia, ajusta según sea necesario
    const float UmbralRot = 5.0;

    float rotAngular = UmbralRot * angle;

    if(lateral_distance < REFERENCE_DISTANCE - 100) {  // Si está demasiado cerca de la pared
        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=rotAngular};
    }
    else if(lateral_distance > REFERENCE_DISTANCE + 100) {  // Si está demasiado lejos de la pared
        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=-rotAngular};
    }
    else {  // Si está a una buena distancia de la pared
        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=0};
    }

    return std::make_tuple(estado, robot_speed);
}










std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::chocachoca(RoboCompLidar3D::TPoints &points) {


    qInfo() << "Estado chocachoca";


    //Sacamos los puntos mínimos.

    int offset = points.size() / 2 - points.size() / 3;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    RobotSpeed robot_speed;
    const float MIN_DISTANCE = 600;


    if (std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE) {
        qInfo() << "Too close to the wall - Avoiding";

        // Move away from the wall
        robot_speed = RobotSpeed{.adv = 0, .side = 0, .rot = 0};
        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);
    } else {  // Continue moving forward
        qInfo() << "Follow movement";
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=0};


//        // Crear un generador de números aleatorios
//        std::mt19937 gen(std::random_device{}());
//
//        // Crear una distribución uniforme entre 1 y 10
//        std::uniform_real_distribution<> distrib(0, 1.0);
//
//        float md = distrib(gen);
//
//        if (md < 0.5) {
//            return std::make_tuple(Estado::SPIRAL, robot_speed);
//
//        }

        return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);
    }
}




//Este funciona
std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::spiral(){
    qInfo()<<"Estado spiral funciona";
    static float rot = 0.3;  // Inicialización de la rotación
    static float adv = 1.0;  // Inicialización de la velocidad de avance
    static float increase_rate = 0.005;  // Tasa de incremento de la rotación

    RobotSpeed robot_speed;

    // Incrementa la rotación en cada llamada a la función
    rot += increase_rate;

    robot_speed = RobotSpeed{.adv = adv, .side = 0, .rot = rot};

    return std::make_tuple(Estado::SPIRAL, robot_speed);
}



int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    //Borramos los gráficos del QT porfa que funcione.
    static std::vector<QGraphicsItem*> borrar;

    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }

    borrar.clear();

    for(const auto &p : points)
    {
        auto point = viewer->scene.addRect(-50,-50,100, 100, QPen(QColor("Blue")), QBrush(QColor("Blue")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
}





/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
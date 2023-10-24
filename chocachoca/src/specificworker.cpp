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
        case Estado::IDLE:
            break;
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




std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::turn(RoboCompLidar3D::TPoints &points) {

    const float FREE_SPACE_THRESHOLD = 400;  // Umbral ajustado para espacio libre
    const float MAX_ADV = 1.0;  // Asume que el avance máximo es 1.0, reemplaza con tu valor

    // Determina si hay espacio libre al frente chequeando el dato láser en frente del robot
    int middle_index = points.size() / 2;
    auto middle_distance = std::hypot(points[middle_index].x, points[middle_index].y);

    Estado state;

    RobotSpeed robot;

    float adv = 1.0;
    float side = robot.side;
    float rot = robot.rot;


    if (middle_distance > FREE_SPACE_THRESHOLD)
    {
        state = Estado::FOLLOW_WALL;  // o State::FOLLOW_WALL dependiendo del comportamiento deseado
        adv = MAX_ADV;
    }
    // Si no hay espacio libre, no se hace nada y se mantiene el signo de giro seleccionado al salir de FORWARD.


    return std::make_tuple(state, RobotSpeed{adv, side , rot});




}




std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points){



    int start_offset = points.size() / 6;
    int end_offset = (points.size() * 2) / 3;


    //Distancias mínimas

    auto min_elem = std::min_element(points.begin() + start_offset, points.begin() + end_offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });


    qInfo()<< "Estado follow wall";

    RobotSpeed robot_speed;
    Estado estado;
    const float MIN_DISTANCE = 600;
    const float REFERENCE_DISTANCE = 300;  // Assume a reference distance, adjust as needed
    const float delta = 1;  // Assume a delta value, adjust as needed

    //float lateral_distance = std::atan2(min_elem->x,min_elem->y);

    float lateral_distance = std::hypot(min_elem->x, min_elem->y);

    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> dist(0, 1.0);



    if(lateral_distance < MIN_DISTANCE) {
        //Volvemos para atrás
        //estado = Estado::TURN;

        qInfo()<< "Menor la distancia lateral";


        estado =  Estado::TURN;
        robot_speed = RobotSpeed{.adv=0.5, .side=0, .rot=((std::distance(points.begin(), min_elem)) < points.size() / 2) ? 0.5 : -0.5};

    }
    else if(lateral_distance < REFERENCE_DISTANCE - delta) {

        // estamos cerca
        estado =  Estado::STRAIGHT_LINE;

        qInfo()<< "DELTA MAYOR";


        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=-0.2};
    }
    else if(lateral_distance > REFERENCE_DISTANCE + delta) {
        //Straigh line derecho
        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=0.2};
    }
    else {
        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=0};
    }

    return std::make_tuple(estado, robot_speed);




}

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::chocachoca(RoboCompLidar3D::TPoints &points){


    qInfo()<< "Estado chocachoca";
    //Random:
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> dist(0, 1.0);


    //Sacamos los puntos mínimos.

    int offset = points.size()/2-points.size()/3;
    auto min_elem = std::min_element(points.begin()+offset, points.end()-offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    RobotSpeed robot_speed;
    const float MIN_DISTANCE = 1000;
//    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
//    {
//        qInfo()<< "Minimo";
//
//        if(dist(mt) > 0.5)  //Tiramos un dado para cambiar a chocachoca STRAIGHT LINE
//            return std::make_tuple(Estado::STRAIGHT_LINE, RobotSpeed{.adv=0, .side=0, .rot=0});
//        else
//            robot_speed = RobotSpeed{.adv= 0.5, .side=0, .rot=0.5};
//
//
//
//
//    }
//    else
//        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=0};


    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        qInfo()<< "Too close to the wall - Avoiding";

        // Move away from the wall
        robot_speed = RobotSpeed{.adv= 0.5, .side=0, .rot=0.5};
        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);



    }
    else
    {
        // Continue moving forward
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=0};
        return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);

    }




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
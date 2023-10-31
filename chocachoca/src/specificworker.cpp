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

        case Estado::IDLE: {
            res = spiral(filtered_points);
            break;
        }
        case Estado::FOLLOW_WALL:
            res = follow_wall(filtered_points);

            break;
        case Estado::STRAIGHT_LINE: {
            res = straight_line(filtered_points);
            break;
        }
        case Estado::SPIRAL: {
            res = spiral(filtered_points);
            break;
        }
        case Estado::MOVE_TO_CENTER: {
            res = move_to_center(filtered_points);
            break;

        }
    }




    //Desempaquetamos la tupla.
    estado = std::get<0>(res);




    RobotSpeed robot_speed_res = std::get<1>(res);
    const auto &[adv, side, rot] = robot_speed_res;
    omnirobot_proxy->setSpeedBase(adv, side, rot);

}

/*
/////////////////////////////////////////////////////////////////////////////////////////////
                                    ESTADOS
/////////////////////////////////////////////////////////////////////////////////////////////
*/

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::turn(RoboCompLidar3D::TPoints &points) {

    qInfo() << "Estado TURN";

    RobotSpeed robot_speed;

    //Habiendo guardado la última rotación de follow_wall en una variable miembro llamada last_rotAngular
    float rotAngular = last_rotAngular;

    robot_speed = RobotSpeed{.adv = 1, .side = 0, .rot = rotAngular};

    return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed); // Devuelve el estado STRAIGHT_LINE
}

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points){

    //qInfo()<< "Estado FOLLOW_WALL";

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
        last_rotAngular = rotAngular;

        robot_speed = RobotSpeed{.adv=2, .side=0, .rot=rotAngular};
    }
    else if(lateral_distance > REFERENCE_DISTANCE + 100) {  // Si está demasiado lejos de la pared
        estado = Estado::STRAIGHT_LINE;
        last_rotAngular = -rotAngular;

        robot_speed = RobotSpeed{.adv=2, .side=0, .rot=-rotAngular};
    }


    else {  // Si está a una buena distancia de la pared
        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv=2, .side=0, .rot=0};


        if (interactions_follow_wall < 25) {
            estado = Estado::STRAIGHT_LINE;

            last_rotAngular = (last_rotAngular > 0) ? 3.0 : -3.0;


            robot_speed = RobotSpeed{.adv=2, .side=0, .rot=last_rotAngular};
        }
        else{

            if (interactions_follow_wall == 25){
                qInfo() << "Hemos llegado a 11 iteracciones";
                return std::make_tuple(estado, robot_speed);
            }

            interactions_follow_wall++;
        }

    }


    return std::make_tuple(estado, robot_speed);
}

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::straight_line(RoboCompLidar3D::TPoints &points)
{

    // qInfo() << "Estado STRAIGHT_LINE";

    static std::mt19937 gen(std::random_device{}());  // Generador de números aleatorios
    static std::uniform_real_distribution<> dis(0, 1);  // Distribución uniforme entre 0 y 1


    //Sacamos los puntos mínimos.

    int offset = points.size() / 2 - points.size() / 3;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    RobotSpeed robot_speed;

    Estado estado;

    const float MIN_DISTANCE = 600;


    if (std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE) {

        //qInfo() << "Too close to the wall - Avoiding, We enter in the FOLLOW_WALL STATE";

        // Move away from the wall

        robot_speed = RobotSpeed{.adv = 0, .side = 0, .rot = 0};

        // Cambiamos a FOLLOW_WALL ya que no tenemos espacio libre.

        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);

    } else {
        // Continue moving forward

        //Modo debug

        if (DEBUG_MODE){
            estado = Estado::STRAIGHT_LINE;

            robot_speed = RobotSpeed{.adv=2, .side=0, .rot=0};

            return std::make_tuple(estado, robot_speed);

        }

        if (interactions_follow_wall == 25){

            qInfo() << "Hemos llegado a 11 iteracciones";
            interactions_follow_wall = 0;
            estado = Estado::SPIRAL;
            robot_speed = RobotSpeed{.adv=2, .side=0, .rot= last_rotAngular};

            return std::make_tuple(estado, robot_speed);




        }

        if (interactions_follow_wall > 0)
        {
            qInfo() << "Vamos a follow wall";

            estado = Estado::FOLLOW_WALL;
            robot_speed = RobotSpeed{.adv = 2, .side = 0, .rot = 0};
            return std::make_tuple(estado, robot_speed);
        }


        if (dosveces >= 10){
            qInfo()<< "straight line otra vez con dos veces";
            dosveces = 0;
            interactions = 0;
            float random = dis(gen);  // Número aleatorio entre 0 y 1

            if (random < 0.5) {
                // 50% de probabilidad de entrar aquí
                float random_rot = dis(gen);  // Número aleatorio entre 0 y 1

                // Decide la dirección de rotación aleatoriamente
                random_rot = (dis(gen) < 0.5) ? 3 : -3;

                estado = Estado::STRAIGHT_LINE;
                robot_speed = RobotSpeed{.adv=2, .side=0, .rot=random_rot};
            }
            else {
                estado = Estado::STRAIGHT_LINE;
                robot_speed = RobotSpeed{.adv=2, .side=0, .rot=0};
            }
        }

        if (interactions >= MAX_INTERACTIONS){

            dosveces++;

            qInfo()<< "straight line otra vez";
            float random = dis(gen);  // Número aleatorio entre 0 y 1

            if (random < 0.5) {
                // 50% de probabilidad de entrar aquí
                float random_rot = dis(gen);  // Número aleatorio entre 0 y 1
                float max_rot = 1.0;  // Asume un valor máximo de rotación de 1.0 (ajústalo según sea necesario)
                random_rot *= max_rot;  // Escala el valor aleatorio al rango de rotación máximo

                // Decide la dirección de rotación aleatoriamente
                random_rot = (dis(gen) < 0.5) ? 1 : -1;

                estado = Estado::STRAIGHT_LINE;
                robot_speed = RobotSpeed{.adv=2, .side=0, .rot=random_rot};
            }
            else {
                estado = Estado::STRAIGHT_LINE;
                robot_speed = RobotSpeed{.adv=2, .side=0, .rot=0};
            }


            return std::make_tuple(estado, robot_speed);


        }


        qInfo() << "Follow movement to SPIRAL";

        robot_speed = RobotSpeed{.adv = 1.0, .side = 0, .rot = 0};
        return std::make_tuple(Estado::SPIRAL, robot_speed);
    }
}

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::spiral(RoboCompLidar3D::TPoints &points)
{

    qInfo() << "Estado SPIRAL";

    static float SPIRAL_SPEED = 1.0;    // Velocidad de avance del robot
    static float SPIRAL_ROTATION = 0.5; // Tasa de rotación inicial - esto determinará el tamaño de la espiral
    static float INCREASE_RATE = 0.01;  // Tasa de incremento de la rotación para hacer espirales más cerradas

    const int MIN_DISTANCE = 600; // Distancia para considerar el cambio al estado FOLLOW_WALL

    RobotSpeed robot_speed;

    int offset = points.size() / 2 - points.size() / 5;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b)
                                     { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });


    if (std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        SPIRAL_ROTATION = 0.5; // Resetear la tasa de rotación

        robot_speed = RobotSpeed{.adv = 0, .side = 0, .rot = 0};
        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);
    }
    else
    {

        if (interactions >= MAX_INTERACTIONS) {

            qInfo ()<<"Hemos superados las iteracciones de spiral";

            robot_speed = RobotSpeed{.adv = SPIRAL_SPEED, .side = 0, .rot = 0};
            return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);

        }

        robot_speed = RobotSpeed{.adv = SPIRAL_SPEED, .side = 0, .rot = SPIRAL_ROTATION};
        SPIRAL_ROTATION += INCREASE_RATE; // Incrementar la tasa de rotación

        interactions++;

        //Devolvemos la tupla
        return std::make_tuple(Estado::SPIRAL, robot_speed);
    }
}

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::move_to_center(RoboCompLidar3D::TPoints &points) {
    int offset = points.size() / 2 - points.size() / 5;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });
    RobotSpeed robot_speed;

    static int i = 0;

    if (std::hypot(min_elem->x, min_elem->y) < 600) {
        robot_speed = RobotSpeed{.adv = 1.0, .side = 0, .rot = -1.0};


    }

    if (i == 25) {

        i = 0;

        return std::make_tuple(Estado::SPIRAL, robot_speed);


    }



    robot_speed = RobotSpeed{.adv = 1.0, .side = 0, .rot = -1.0};

    i++;
    qInfo()<< i;
    return std::make_tuple(Estado::MOVE_TO_CENTER, robot_speed);

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


std::tuple<bool, bool> SpecificWorker::checkFreeSpace(RoboCompLidar3D::TPoints &points, float min_distance)
{
    if(points.empty()) return std::make_tuple(false, false);

    float leftMinDistance = std::numeric_limits<float>::max();
    float rightMinDistance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < points.size(); ++i)
    {
        float distance = std::hypot(points[i].x, points[i].y);
        float angle = std::atan2(points[i].y, points[i].x);

        if (angle > 0 && angle <= M_PI/2)  // Lado izquierdo
        {
            leftMinDistance = std::min(leftMinDistance, distance);
        }
        else if (angle >= -M_PI/2 && angle < 0)  // Lado derecho
        {
            rightMinDistance = std::min(rightMinDistance, distance);
        }
    }

    bool leftFree = (leftMinDistance > min_distance);
    bool rightFree = (rightMinDistance > min_distance);

    return std::make_tuple(leftFree, rightFree);
}


/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
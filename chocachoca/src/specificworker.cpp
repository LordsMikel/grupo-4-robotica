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

    // Assuming you have a member variable to count the iterations
    static int st_iterations = 0;
    const int MAX_FOLLOW_WALL_ITERATIONS = 50;  // Change this value as needed


    switch(estado)
    {
        //Cambiar después de IDLE:
        case Estado::IDLE: {
            res = spiral(filtered_points);
            break;
        }

        case Estado::CHECK_WALL: {

            res = check_wall(filtered_points);
            break;

        }

        case Estado::FOLLOW_WALL:
            res = follow_wall(filtered_points);

            break;
        case Estado::STRAIGHT_LINE: {
            res = chocachoca(filtered_points);
            st_iterations++;

            break;
        }
        case Estado::TURN: {
            res = turn(filtered_points);
            break;
        }

        case Estado::SPIRAL: {

            res = spiral(filtered_points);
            break;

        }
    }




    //Desempaquetamos la tupla.
    estado = std::get<0>(res);

    qInfo() << st_iterations;

    if(st_iterations >= MAX_FOLLOW_WALL_ITERATIONS) {
        estado = Estado::SPIRAL;  // Transition to SPIRAL state
        st_iterations = 0;  // Reset the counter
    }



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

    heVenido = true;

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

        robot_speed.adv = 0;
        robot_speed.side = 0;
        robot_speed.rot = 0;

        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);
    } else {  // Continue moving forward
        qInfo() << "Follow movement";
        robot_speed.adv = 1.0;
        robot_speed.side = 0;
        robot_speed.rot = 0;
        return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);  // Cambio aquí    }
    }


}




std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::check_wall(RoboCompLidar3D::TPoints &points) {
    qInfo() << "Estado check_wall";

    // Identifica el punto más cercano en el rango frontal
    int offset = points.size() / 2 - points.size() / 3;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    const float MIN_DISTANCE = 600;



    qInfo()<<"No venimos de follow wall";




    if (std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE) {
        qInfo() << "Too close to the wall - Avoiding";
        return std::make_tuple(Estado::FOLLOW_WALL, RobotSpeed{0, 0, 0});
    } else {
        // Generar un número aleatorio para decidir el próximo estado
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<> distrib(0, 1.0);
        float random_value = distrib(gen);

        if (!heVenido) {

            qInfo()<< "Vamos a spiral";
            RobotSpeed robotSpeed;
            heVenido = true;
            return std::make_tuple(Estado::SPIRAL, robotSpeed);

        } else if (random_value < 0.66) {
            qInfo()<<"Vamos a linea recta";
            RobotSpeed robotSpeed;

            robotSpeed.adv = 1.0;
            robotSpeed.side = 0;
            robotSpeed.rot = 0;



            return std::make_tuple(Estado::STRAIGHT_LINE, robotSpeed);
        } else {

            qInfo()<< "vamos al estado follow wall manteniendo los valores";
            RobotSpeed robotSpeed;




            return std::make_tuple(Estado::FOLLOW_WALL, robotSpeed);
        }
    }
}




//Este funciona
std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::spiral(RoboCompLidar3D::TPoints &points){
    qInfo()<<"Estado spiral funciona";
    static float rot = 0.3;  // Inicialización de la rotación
    static float adv = 1.0;  // Inicialización de la velocidad de avance
    static float increase_rate = 0.01;  // Tasa de incremento de la rotación
    static int spiral_iterations = 0;  // Contador de iteraciones en el estado SPIRAL
    const int MAX_SPIRAL_ITERATIONS = 200;  // Número máximo de iteraciones en el estado SPIRAL

    // Definir los índices para los rangos del lado derecho
    int start_offset_right = points.size() / 6;
    int end_offset_right =  (points.size() * 2) / 3;

    // Encontrar el punto más cercano en los rangos del lado derecho
    auto min_elem_right = std::min_element(points.begin() + start_offset_right, points.end() - end_offset_right,
                                           [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    float angle = std::atan2(min_elem_right->y, min_elem_right->x);  // Calcula el ángulo del punto más cercano
    float lateral_distance = std::hypot(min_elem_right->x, min_elem_right->y);  // Calcula la distancia lateral

    RobotSpeed robot_speed;
    Estado estado;
    const float REFERENCE_DISTANCE = 900;  // Asume una distancia de referencia, ajusta según sea necesario
    const float UmbralRot = 5.0;

    float rotAngular = UmbralRot * angle;

    // Incrementa el contador de iteraciones en el estado SPIRAL
    spiral_iterations++;


    int offset = points.size() / 2 - points.size() / 3;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });


    const float MIN_DISTANCE = 85;
    // Use the function in your logic

    qInfo()<< "MIN es 85";

    if (!isStraightWallStretch(points, MIN_DISTANCE, points.size() / 6)) {

        robot_speed.adv = 0;
        robot_speed.side = 0;
        robot_speed.rot = 0;
        heVenido = false;
        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);
    }

    qInfo() << spiral_iterations;

    if(spiral_iterations >= MAX_SPIRAL_ITERATIONS) {

        heVenido = true;
        // Si se alcanza el número máximo de iteraciones, cambia al estado CHECK_WALL
        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv=0, .side=0, .rot=0};  // Detén el robot antes de cambiar de estado
        spiral_iterations = 0;  // Restablece el contador de iteraciones
        return std::make_tuple(estado, robot_speed);
    }

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


// Define a function to check if there's a straight stretch of wall to follow
bool SpecificWorker::isStraightWallStretch(const RoboCompLidar3D::TPoints &points, float min_distance,
                                           int range_offset) {
    for (int i = points.size() / 2 - range_offset; i < points.size() / 2 + range_offset; ++i) {
        if (std::hypot(points[i].x, points[i].y) < min_distance) {
            return false;  // A point too close to the robot found, not a straight stretch
        }
    }
    return true;  // All points in the range are at a safe distance, it's a straight stretch
}


/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
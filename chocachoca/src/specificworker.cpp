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
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
    // Uncomment if there's too many debug messages
    // but it removes the possibility to see the messages
    // shown in the console with qDebug()
    //	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
 * \brief Default destructor
 */
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
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

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if (this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(460, 480, 0, 100, QColor("Blue"));
        viewer->show();
        viewer->activateWindow();
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{

    RoboCompLidar3D::TData ldata;

    ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1);
    const auto &points = ldata.points;
    if (points.empty())
        return;

    RoboCompLidar3D::TPoints filtered_points;
    std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p)
    { return p.z < 2000; });

    draw_lidar(filtered_points, viewer);
    std::tuple<Estado, RobotSpeed> res;

    switch (estado)
    {

        case Estado::IDLE:
        {
            res = spiral(filtered_points);
            break;
        }
        case Estado::FOLLOW_WALL:
            res = follow_wall(filtered_points);

            break;
        case Estado::STRAIGHT_LINE:
        {
            res = straight_line(filtered_points);
            break;
        }
        case Estado::SPIRAL:
        {
            res = spiral(filtered_points);
            break;
        }
        case Estado::MOVE_TO_CENTER:
        {
            res = move_to_center(filtered_points);
            break;
        }
    }

    // Desempaquetamos la tupla.
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

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points)
{

    // qInfo()<< "Estado FOLLOW_WALL";

    static std::mt19937 gen(std::random_device{}());   // Generador de números aleatorios
    static std::uniform_real_distribution<> dis(0, 1); // Distribución uniforme entre 0 y 1

    // Define los índices de inicio y fin para el rango lateral
    int start_offset_lat = points.size() / 6;
    int end_offset_lat = (points.size() * 2) / 3;

    // Encuentra el punto más cercano en el rango lateral
    auto min_elem_lat = std::min_element(points.begin() + start_offset_lat, points.end() - end_offset_lat,
                                         [](auto a, auto b)
                                         { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    // Calcula el ángulo y la distancia al punto más cercano
    float angle = std::atan2(min_elem_lat->y, min_elem_lat->x);
    float lateral_distance = std::hypot(min_elem_lat->x, min_elem_lat->y);

    RobotSpeed robot_speed;
    Estado estado;

    const float UmbralRot = 7.0;




    float rotAngular = UmbralRot * angle;

    qInfo()<< "La distancia de referencia es: " << REFERENCE_DISTANCE;

        if (lateral_distance < REFERENCE_DISTANCE - 100) {
         // Si está demasiado cerca de la pared
            estado = Estado::STRAIGHT_LINE;
            last_rotAngular = rotAngular;

            //si no funciona ponlo como rotAngular

            robot_speed = RobotSpeed{.adv = 2, .side = 0, .rot = last_rotAngular};
        }
        else if (lateral_distance > REFERENCE_DISTANCE + 100) {
            estado = Estado::STRAIGHT_LINE;
            last_rotAngular = -rotAngular;

            //si no funciona ponlo como rotAngular

            robot_speed = RobotSpeed{.adv = 2, .side = 0, .rot = last_rotAngular};
        }
        else {

        // Si está a una buena distancia de la pared

        estado = Estado::STRAIGHT_LINE;
        robot_speed = RobotSpeed{.adv = 2, .side = 0, .rot = 0};



        //Podríamos entrar a espiral a veces!!!




            // utilizamos setSpeedBase para que el robot gire con last_rotAngular
            // Al utiliza omnirobot_proxy->setSpeedBase(adv, side, rot) el robot gira con rot
            // Y nos ahorramos un estado TURN.

            // Si entramos aquí haremos que todo lo haga el 20% de las veces


//
//            if (dis(gen) < 0.2) {
//
//                qInfo()<< "Entramos en random";
//
//                qInfo() << "Buenas:   " << last_rotAngular;
//
//                omnirobot_proxy->setSpeedBase(2.0, 0, last_rotAngular);
//
//                // Cambiamos a SPIRAL
//                return std::make_tuple(Estado::SPIRAL, robot_speed);
//            }


    }

    return std::make_tuple(estado, robot_speed);
}

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::straight_line(RoboCompLidar3D::TPoints &points)
{

    // qInfo() << "Estado STRAIGHT_LINE";

    static std::mt19937 gen(std::random_device{}());   // Generador de números aleatorios
    static std::uniform_real_distribution<> dis(0, 1); // Distribución uniforme entre 0 y 1

    // Sacamos los puntos mínimos.

    int offset = points.size() / 2 - points.size() / 3;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b)
                                     { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    RobotSpeed robot_speed;

    Estado estado = Estado::STRAIGHT_LINE;

    static int i = 0;
    static int stop = 0;

    // Calculate the distance from the robot to the nearest point on the wall
    float distance_to_wall = std::hypot(min_elem->x, min_elem->y);



    qInfo () << "Threshold is: "<< REFERENCE_DISTANCE;

    if (std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {

        // qInfo() << "Too close to the wall - Avoiding, We enter in the FOLLOW_WALL STATE";

        // Move away from the wall

    // Si está demasiado cerca de la pared, cambia a FOLLOW_WALL y reduce la REFERENCE_DISTANCE en 100
        if (stop != 1) {
            REFERENCE_DISTANCE += 100;
        }


        robot_speed = RobotSpeed{.adv = 0, .side = 0, .rot = 0};

        // Cambiamos a FOLLOW_WALL ya que no tenemos espacio libre.

        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);
    }

    // Si ya tenemos la distancia de referencia a 900, cambiamos a SPIRAL a veces.
    if (std::hypot(min_elem->x, min_elem->y) > REFERENCE_DISTANCE) {

        qInfo() << "segundo if";
        if (stop != -1) {
            REFERENCE_DISTANCE += 100;
        }

        //omnirobot_proxy->setSpeedBase(2,0,0);

        robot_speed = RobotSpeed{.adv = 2.0, .side = 0, .rot = 0};
        return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);



    }
    else{
        qInfo() << "tercer if";


        i++;

        qInfo()<< "Valor de la i" << i;

        if (i == 100 && REFERENCE_DISTANCE == 2500 && stop != -1){

            REFERENCE_DISTANCE = 900;

            stop = -1;

            i = 0;

        }

        if (i == 100 && stop != -1) {

            REFERENCE_DISTANCE += 100;

            i = 0;

        }

        robot_speed = RobotSpeed{.adv = 2.0, .side = 0, .rot = 0};

        // Cambiamos a FOLLOW_WALL ya que no tenemos espacio libre.

        return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);

    }


//
//        // con dis(gen) < 0.2 tenemos un 20% de probabilidad de entrar en SPIRAL
//        if (dis(gen) < 0.2)
//        {
//            // Cambiamos a SPIRAL
//            qInfo() << "Cambio a SPIRAL";
//            qInfo() << "La distancia de referencia es: " << REFERENCE_DISTANCE;
//            qInfo() << "La distancia del punto más cercano es: " << std::hypot(min_elem->x, min_elem->y);
//            qInfo() << DEBUG_MODE;
//
//            return std::make_tuple(Estado::SPIRAL, robot_speed);
//        }
//
//        return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);
//
//
//    }



}



std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::spiral(RoboCompLidar3D::TPoints &points)
{
    //qInfo() << "Estado SPIRAL";

    static float SPIRAL_SPEED = 1.0;    // Velocidad de avance del robot
    static float SPIRAL_ROTATION = 0.5; // Tasa de rotación inicial
    static float INCREASE_RATE = 0.01;  // Tasa de incremento de la rotación
    static bool clockwise = true;       // Dirección de la espiral (true para horario, false para antihorario)




    RobotSpeed robot_speed;

    int offset = points.size() / 2 - points.size() / 5;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b)
                                     { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    if (std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        SPIRAL_ROTATION = 0.5;  // Reset rotation rate

        clockwise = !clockwise; // Invert the spiral direction

        qInfo() << "Espiral a FOLLOW WALL";


        return std::make_tuple(Estado::FOLLOW_WALL, RobotSpeed{.adv = 0, .side = 0, .rot = 0});
    }
    else
    {
        if (interactions >= MAX_INTERACTIONS)
        {
            interactions = 0;

            SPIRAL_ROTATION = 0.5;  // Reset rotation rate

            clockwise = !clockwise; // Invert the spiral direction

            return std::make_tuple(Estado::STRAIGHT_LINE, RobotSpeed{.adv = SPIRAL_SPEED, .side = 0, .rot = 0});
        }

        robot_speed = RobotSpeed{.adv = SPIRAL_SPEED, .side = 0, .rot = (clockwise ? 1 : -1) * SPIRAL_ROTATION};
        SPIRAL_ROTATION += INCREASE_RATE;

        interactions++;

        return std::make_tuple(Estado::SPIRAL, robot_speed);
    }
}

std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::move_to_center(RoboCompLidar3D::TPoints &points)
{
    int offset = points.size() / 2 - points.size() / 5;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset,
                                     [](auto a, auto b)
                                     { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });
    RobotSpeed robot_speed;

    static int i = 0;

    if (std::hypot(min_elem->x, min_elem->y) < 600)
    {
        robot_speed = RobotSpeed{.adv = 1.0, .side = 0, .rot = -1.0};
    }

    if (i == 5)
    {

        i = 0;

        return std::make_tuple(Estado::SPIRAL, robot_speed);
    }

    robot_speed = RobotSpeed{.adv = 1.0, .side = 0, .rot = -1.0};

    i++;
    qInfo() << i;
    return std::make_tuple(Estado::MOVE_TO_CENTER, robot_speed);
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    // Borramos los gráficos del QT porfa que funcione.
    static std::vector<QGraphicsItem *> borrar;

    for (auto &b : borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }

    borrar.clear();

    for (const auto &p : points)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("Blue")), QBrush(QColor("Blue")));
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
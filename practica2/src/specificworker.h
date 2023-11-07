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

/**
    \brief
    @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>

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

    // estructura de acoplamiento del robot

    struct RobotSpeed
    {
        float adv = 0, side = 0, rot = 0;
    };

    void draw_lidar(RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);

    // Estados
    enum class Estado
    {
        IDLE,
        FOLLOW_WALL,
        STRAIGHT_LINE,
        SPIRAL

    };


    Estado estado = Estado::SPIRAL;

    // Tuplas para acoplar los estados:

    std::tuple<Estado, RobotSpeed> straight_line(RoboCompLidar3D::TPoints &points);

    std::tuple<Estado, RobotSpeed> follow_wall(RoboCompLidar3D::TPoints &points);


    std::tuple<Estado, RobotSpeed> spiral(RoboCompLidar3D::TPoints &points);

    std::tuple<Estado, RobotSpeed> move_to_center(RoboCompLidar3D::TPoints &points);



    float last_rotAngular = 0.0;


    float REFERENCE_DISTANCE = 900;  // Asume una distancia de referencia.


    const int MIN_DISTANCE = 600; // Distancia minima para que el robot se pare

    bool prueba = false;

    int MAX_INTERACTIONS = 200;


    bool change = false;


    bool reset = false;

    


    int interactions = 0;

    bool DEBUG_MODE = false;

    //Creamos un booleano si el robot su distancia frontal es mayor que la REFERENCE_DISTANCE
    bool front_distance = false;


    int velocidad = 0;

    bool hemosVenido = false;

    bool clockwise = true;       // Dirección de la espiral (true para horario, false para antihorario)


};

#endif

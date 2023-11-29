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
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>


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
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        // Inicializaciones personales
        viewer = new AbstractGraphicViewer(this, QRectF(-5000,-5000,10000,10000));
        viewer->add_robot(460,480,0,100,QColor("Blue"));
        viewer->show();
        viewer->activateWindow();

        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    RoboCompLidar3D::TData ldata;
    ldata = lidar3d_proxy->getLidarData("helios", 0, 360, 1);
    const auto &points = ldata.points;
    if (points.empty()) return;

    // filter
    RoboCompLidar3D::TPoints filtered_points;
    std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) { return p.z < 2000; });

    // doors
    auto doors = doors_extractor(filtered_points);

    // state machine
    switch (state) {
        case States::IDLE:
        {
            move_robot(0,0,0);
            break;
        }
        case States::SEARCH_DOOR:
        {
            //qInfo() << "SEARCH_DOOR";
            if(not doors.empty())
            {
                door_target = doors[0];
                move_robot(0,0,0);
                state = States::GOTO_DOOR;
                qInfo() << "First found";
                door_target.print();
            }
            else
                move_robot(0,0,0.5);
            break;
        }
        case States::GOTO_DOOR:
        {
            //Info() << "GOTO_DOOR";
            if(door_target.dist_to_robot() < DOOR_PROXIMITY_THRESHOLD)
            {
                qInfo() << "distance " << door_target.dist_to_robot();
                move_robot(0,0,0);
                qInfo() << "GOTO_DOOR Target achieved";
                state = States::IDLE;
            }
            // match door_target against new perceived doors
            auto res = std::ranges::find(doors, door_target);
            if( res != doors.end())
            {
                door_target = *res;
                float rot = -0.5*door_target.angle_to_robot();
                float adv = MAX_ADV_SPEED * break_adv(door_target.dist_to_robot()) * break_rot(door_target.angle_to_robot()) /1000.f;
                move_robot(0, adv, rot);
            }
            else
            {
                move_robot(0,0,0);
                state = States::SEARCH_DOOR;
                qInfo() << "GOTO_DOOR Door lost, searching";
            }
         break;
        }
    }

    // move the robot

}
///////////////////////////////////////////////////////////////////////////////
float SpecificWorker::break_adv(float dist_to_target)
{
    return std::clamp(dist_to_target/DOOR_PROXIMITY_THRESHOLD, 0.f, 1.f );
}
float SpecificWorker::break_rot(float rot)
{
    if(rot>=0)
        return std::clamp(1-rot, 0.f, 1.f);
    else
        return std::clamp(rot+1, 0.f, 1.f);
}


void SpecificWorker::move_robot(float side, float adv, float rot)
{
    try
    {
        omnirobot_proxy->setSpeedBase(adv, 0, rot);
    }
    catch(const Ice::Exception &e){ std::cout << e << std::endl;}
}
SpecificWorker::Doors
SpecificWorker::doors_extractor(const RoboCompLidar3D::TPoints  &filtered_points)
{
    auto lines = extract_lines(filtered_points);
    auto peaks = extract_peaks(lines);
    auto doors = get_doors(peaks);
    auto final_doors = filter_doors(doors);

    draw_lidar(lines.middle, viewer);
    draw_doors(final_doors, viewer);
    //draw_doors(std::get<0>(doors), viewer, QColor("purple"));
    //draw_doors(std::get<1>(doors), viewer, QColor("red"));
    //draw_doors(std::get<2>(doors), viewer, QColor("magenta"));
    return final_doors;
}

SpecificWorker::Lines SpecificWorker::extract_lines(const RoboCompLidar3D::TPoints &points)
{
    Lines lines;
    for(const auto &p: points)
    {
        //qInfo() << p.x << p.y << p.z;
        if(p.z > LOW_LOW and p.z < LOW_HIGH)
            lines.low.push_back(p);
        if(p.z > MIDDLE_LOW and p.z < MIDDLE_HIGH)
            lines.middle.push_back(p);
        if(p.z > HIGH_LOW and p.z < HIGH_HIGH)
            lines.high.push_back(p);
    }
    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_peaks(const SpecificWorker::Lines &lines) {
    Lines peaks;
    const float THRES_PEAK = 1000;

    for (const auto &both: iter::sliding_window(lines.low, 2))
        if (fabs(both[1].r - both[0].r) > THRES_PEAK) {
            if (both[0].r < both[1].r) peaks.low.push_back(both[0]);
            else peaks.low.push_back(both[1]);
        }

    for (const auto &both: iter::sliding_window(lines.middle, 2))
        if (fabs(both[1].r - both[0].r) > THRES_PEAK) {
            if (both[0].r < both[1].r) peaks.middle.push_back(both[0]);
            else peaks.middle.push_back(both[1]);
        }

    for(const auto &both: iter::sliding_window(lines.high, 2))
        if(fabs(both[1].r - both[0].r) > THRES_PEAK) {
            if (both[0].r < both[1].r) peaks.high.push_back(both[0]);
            else peaks.high.push_back(both[1]);
        }

    return peaks;
}

std::tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors>
SpecificWorker::get_doors(const SpecificWorker::Lines &peaks) {

    Doors doors_low, doors_middle, doors_high;

    auto dist = [](auto a, auto b){
        //qInfo() << std::hypot(a.x-b.x, a.y-b.y);
        return std::hypot(a.x-b.x, a.y-b.y);
    };

    const float THRES_DOOR = 500;

    auto near_door = [dist, THRES_DOOR](auto &doors, auto d){
        for(auto &&old: doors)
        {
            //qInfo() << dist(old.left, d.left) << dist(old.right, d.right) << dist(old.right, d.left) << dist(old.left, d.right);
            if( dist(old.left, d.left) < THRES_DOOR or
                dist(old.right, d.right) < THRES_DOOR or
                dist(old.right, d.left) < THRES_DOOR or
                dist(old.left, d.right) < THRES_DOOR)
                return true;
        }
        return false;
    };

    for(auto &par : peaks.low | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_low, door)) {
                doors_low.emplace_back(par[0], par[1]);
            }
        }
    }
    for(auto &par : peaks.middle | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_middle, door)) {
                doors_middle.emplace_back(par[0], par[1]);
            }
        }
    }
    for(auto &par : peaks.high | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_high, door)) {
                doors_high.emplace_back(par[0], par[1]);
            }
        }
    }

    return std::make_tuple(doors_low, doors_middle, doors_high);
}

SpecificWorker::Doors
SpecificWorker::filter_doors(const tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors> &doors)
{
    Doors final_doors;

    auto &[dlow, dmiddle, dhigh] = doors;
//    for(auto &dl: dlow)
//    {
//        std::apply([dl](auto&&... args)
//            {((std::ranges::find(args, dl) != args.end()), ...);}, doors);
//    }
    for(auto &dl: dlow)
    {
        bool equal_middle = std::ranges::find(dmiddle, dl) != dmiddle.end();
        bool equal_high = std::ranges::find(dhigh, dl) != dhigh.end();

        if (equal_middle and equal_high)
            final_doors.push_back(dl);
    }
    return final_doors;
}

////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
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

void SpecificWorker::draw_doors(const Doors &doors, AbstractGraphicViewer *viewer, QColor color)
{
    static std::vector<QGraphicsItem *> borrar;
    for (auto &b: borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    QColor target_color;
    for (const auto &d: doors)
    {
        if(d == door_target)
            target_color = QColor("magenta");
        else
            target_color = color;
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(target_color), QBrush(target_color));
        point->setPos(d.left.x, d.left.y);
        borrar.push_back(point);
        point = viewer->scene.addRect(-50, -50, 100, 100, QPen(target_color), QBrush(target_color));
        point->setPos(d.right.x, d.right.y);
        borrar.push_back(point);
        auto line = viewer->scene.addLine(d.left.x, d.left.y, d.right.x, d.right.y, QPen(target_color, 50));
        borrar.push_back(line);
    }
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

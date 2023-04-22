#include "RobotConfig.h"

RobotConfig::RobotConfig(Hardware* hardware)
{
    hw = hardware;
}

void RobotConfig::setTeamColor(std::pair<double, double> locale){
    teamColor = (locale.second < -locale.first) ? vex::color::red : vex::color::blue;
    setQuadrant(locale);
}

void RobotConfig::setQuadrant(std::pair<double, double> locale){
    if(locale.first > 0 && locale.second > 0){
        quadrant = 1;
    }else if(locale.first < 0 && locale.second > 0){
        quadrant = 2;
    }else if(locale.first > 0 && locale.second < 0){
        quadrant = 3;
    }else{
        quadrant = 4;
    }
    return;
}

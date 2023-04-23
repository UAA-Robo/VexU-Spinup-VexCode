#pragma once
#include "GameElement.h"

/*
    A Frisbee Disk Goal
*/
class FrisbeeGoal: public GameElement{
private:
    int maxCapacity;
    int currentCapacity;
    double radius;
    double height;
    
public:
    /// @brief Constuctor for the frisbee disk goals
    /// @param id Unique Identifier for Game Element
    /// @param x X Component Position
    /// @param y Y Component Position
    /// @param teamColor Side of field the disk is on
    /// @param radius Radius of the disk
    /// @param height Height of Goal
    /// @param maxCapacity Maximum Capacity of the goal
    FrisbeeGoal(
        int id, 
        double x, 
        double y, 
        char teamColor, 
        double radius = 0, 
        double height = 2, 
        int maxCapacity = 12
        ): GameElement(id, x, y, 0, teamColor, 'F', true, 0, radius, 0, radius){
            
            this->maxCapacity = maxCapacity;
            this->height = height;
            this->radius = radius;
            this->currentCapacity = 0;
            this->setIsBackAligned(true);
    }

    // Adds a disk to the goal
    bool AddDisk(){
        if(currentCapacity != maxCapacity){
            currentCapacity++;
            return true;
        }else{
            return false;
        }
    }

    // Removes a disk from the goal
    bool RemoveDisk(){
        if(currentCapacity != 0){
            currentCapacity--;
            return true;
        }else{
            return false;
        }
    }

    // Getters
    double GetHeight(){ return height; }
    double GetRadius(){ return radius; }
    int getCapacity(){ return maxCapacity; }
    int GetCurrentCapacity(){ return currentCapacity; }
};
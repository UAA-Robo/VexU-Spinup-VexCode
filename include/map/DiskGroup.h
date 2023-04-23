#pragma once
#include "GameElement.h"

/*
    Disk Groups are ways to define stacks of disks. A single disk will have a group size of 1 and a stack of 3 of size 3
*/
class DiskGroup: public GameElement{
private:
    double radius;
    int maxCapacity;
    int currentCapacity;
    
public:
    /// @brief Disk Group Constructor
    /// @param id Unique Game Element Identifier
    /// @param x X Component Position
    /// @param y Y Component Position
    /// @param teamColor Side of field the disk is on
    /// @param radius Radius of the disk
    /// @param currentCapacity Current number of disks in the stack
    /// @param maxCapacity Maximum number of disks in the stack
    DiskGroup(int id, double x, double y, char teamColor, double radius, int currentCapacity, int maxCapacity = 3): 
        GameElement(id, x, y, 0, teamColor, 'D', false, 0, radius, 0, radius){
            this->currentCapacity = currentCapacity;
            this->maxCapacity = maxCapacity;
            this->radius = radius;
    }
    // Adds a disk to the group
    bool AddDisk(){
        if(currentCapacity != maxCapacity){
            currentCapacity++;
            return true;
        }else{
            return false;
        }
    }

    // Removes a disk from the group
    bool RemoveDisk(){
        if(currentCapacity != 0){
            currentCapacity--;
            return true;
        }else{
            return false;
        }
    }

    int GetCurrentCapacity(){ return this->currentCapacity; }
    int GetMaxCapacity(){ return this->maxCapacity; }
    double GetRadius(){ return this->radius; }
};
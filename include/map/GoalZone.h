#pragma once
#include "GameElement.h"
#include "DiskGroup.h"
/*
    The Goal Zone that sits just below the frisbee disk goals
*/
class GoalZone: public GameElement{
private:
    int maxCapacity;
    int currentCapacity;
    // Bounding Box of the goal zone
    dbpair ULCorner; // Upper Left Corner
    dbpair LRCorner; // Lower Right Corner
public:
    /// @brief Goal Zone Constructor
    /// @param id Unique Identifier for Game Elements
    /// @param ULX Upper Left Corner X Component
    /// @param ULY Upper Left Corner Y Component
    /// @param LRX Lower Right Corner X Component
    /// @param LRY Lower Right Corner Y Component
    /// @param maxCapacity Maximum Goal Capacity
    /// @param teamColor Side of the field the goal lays on
    GoalZone(int id, double ULX, double ULY, double LRX, double LRY, int maxCapacity, char teamColor)
    :GameElement(id, (ULX + LRX) / 2.0, (ULY + LRY) / 2.0, 0, teamColor, 'Z'){
        // Center of Goal Zone is the position of the game element
        this->ULCorner = dbpair(ULX, ULY);
        this->LRCorner = dbpair(LRX, LRY);
        this->maxCapacity = maxCapacity;
        this->currentCapacity = 0;
    }
    /// @brief Goal Zone Constructor
    /// @param id Unique Identifier for Game Elements
    /// @param upperLeft Upper Left Corner's XY Position
    /// @param lowerRight Lower Right Corner's XY Position
    /// @param maxCapacity Maximum Goal Capacity
    /// @param teamColor Side of the field the goal lays on
    GoalZone(int id, dbpair upperLeft, dbpair lowerRight, int maxCapacity, char teamColor)
    :GameElement(id, (upperLeft.first + lowerRight.first) / 2.0, (upperLeft.second + lowerRight.second) / 2.0, 0, teamColor, 'Z'){
        this->ULCorner = upperLeft;
        this->LRCorner = lowerRight;
        this->maxCapacity = maxCapacity;
        this->currentCapacity = 0;
    }
    // Checks if the given point is in the goal.
    bool IsPointInside(double x, double y){
        return x >= ULCorner.first && x <= LRCorner.first && y <= ULCorner.second && y >= LRCorner.second;
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
    int GetMaxCapacity(){ return maxCapacity; }
    int GetCurrentCapacity(){ return currentCapacity; }
    
    // Corners of the bounding box
    dbpair GetULCorner(){ return ULCorner; }
    dbpair GetLLCorner(){ return dbpair(ULCorner.first, LRCorner.second); }
    dbpair GetURCorner(){ return dbpair(LRCorner.first, ULCorner.second); }
    dbpair GetLRCorner(){ return LRCorner; }
};
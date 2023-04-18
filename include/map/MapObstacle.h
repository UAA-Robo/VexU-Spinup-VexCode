#pragma once
#include "GameElement.h"
/*
    This class is intended to be used to act as a building block for game elements.
*/
class Obstacle: public GameElement{
private:
    dbpair ULCorner; // Upper Left Corner
    dbpair LRCorner; // Lower Right Corner
    
public:
    /// @brief Game Element Constructor
    /// @param id Unique Identification for Game Element
    /// @param ULX Upper Left Corner X Position
    /// @param ULY Upper Left Corner Y Position
    /// @param LRX Lower Right Corner X Position
    /// @param LRY Lower Right Corner Y Position
    Obstacle(int id, double ULX, double ULY, double LRX, double LRY)
    :GameElement(id, (ULX + LRX) / 2.0, (ULY + LRY) / 2.0, 0, 'M', 'O'){
        this->ULCorner = dbpair(ULX, ULY);
        this->LRCorner = dbpair(LRX, LRY);
    }
    /// @brief Game Element Constructor
    /// @param id Unique Identifier for Game Elements
    /// @param upperLeft Upper Left Corner XY Coordinates
    /// @param lowerRight Lower Right Corner XY Coordinates
    Obstacle(int id, dbpair upperLeft, dbpair lowerRight)
    :GameElement(id, (upperLeft.first + lowerRight.first) / 2.0, (upperLeft.second + lowerRight.second) / 2.0, 0, 'M', 'O', 0, 0){
        this->ULCorner = upperLeft;
        this->LRCorner = lowerRight;
    }

    // Checks if the given point is in the goal.
    bool IsPointInside(double x, double y){
        return x >= ULCorner.first && x <= LRCorner.first && y <= ULCorner.second && y >= LRCorner.second;
    }

    // Corners of the bounding box
    dbpair GetULCorner(){ return ULCorner; }
    dbpair GetURCorner(){ return dbpair(LRCorner.first, ULCorner.second); }
    dbpair GetLLCorner(){ return dbpair(ULCorner.first, LRCorner.second); }
    dbpair GetLRCorner(){ return LRCorner; }
};
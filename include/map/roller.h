#pragma once
#include "GameElement.h"
/*
    Roller that sits at the edge of the field
*/
class Roller: public GameElement{
private:
    char initialColor;
    char currentColor;
    std::pair<double, double> ULCorner; // Upper Left Corner
    std::pair<double, double> LRCorner; // Lower Right Corner
public:
    /// @brief Roller Constructor
    /// @param id Unique Identifier for Game Element
    /// @param initialColor Initial Color the Roller is Set to
    /// @param ULX Upper Left Corner X Component
    /// @param ULY Upper Left Corner Y Component
    /// @param LRX Lower Right Corner X Component
    /// @param LRY Lower Right Corner Y Component
    Roller(int id, char initialColor, double ULX, double ULY, double LRX, double LRY)
    :GameElement(id, (ULX + LRX) / 2.0, (ULY + LRY) / 2.0, 0, 'M', 'R'){
        // Center of Goal Zone is the position of the game element
        this->ULCorner = dbpair(ULX, ULY);
        this->LRCorner = dbpair(LRX, LRY);
        this->initialColor = initialColor;
        this->currentColor = initialColor;
    }
    /// @brief Roller Constructor
    /// @param id Unique Identifier for Game Element
    /// @param initialColor Initial Color the Roller is Set to
    /// @param upperLeft Upper Left Corner XY Position
    /// @param lowerRight Lower Right Corner XY Position
    Roller(int id, char initialColor, dbpair upperLeft, dbpair lowerRight)
    :GameElement(id, (upperLeft.first + lowerRight.first) / 2.0, (upperLeft.second + lowerRight.second) / 2.0, 0, 'M', 'R'){
        this->ULCorner = upperLeft;
        this->LRCorner = lowerRight;
        this->initialColor = initialColor;
        this->currentColor = initialColor;
    }

    // Checks if the given point is in the goal.
    bool IsPointInside(double x, double y){
        return x >= ULCorner.first && x <= LRCorner.first && y <= ULCorner.second && y >= LRCorner.second;
    }

    void SetColor(char currentColor){ this->currentColor = currentColor; }
    void resetColor(){ this->currentColor = initialColor; }
    char GetCurrentColor(){ return currentColor; }

    // Corners of the bounding box
    
    dbpair GetULCorner(){ return ULCorner; }
    dbpair GetURCorner(){ return dbpair(LRCorner.first, ULCorner.second); }
    dbpair GetLLCorner(){ return dbpair(ULCorner.first, LRCorner.second); }
    dbpair GetLRCorner(){ return LRCorner; }

};
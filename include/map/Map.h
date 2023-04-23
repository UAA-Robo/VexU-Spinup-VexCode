#pragma once
#include <vector>
#include <string>

#include "GameElement.h"
#include "DiskGroup.h"
#include "FrisbeeGoal.h"
#include "GoalZone.h"
#include "MapObstacle.h"
#include "Roller.h"

/*
    Representation of the Map
*/
class Map{
    private:
        int nextElementID;
        void addObstacle(dbpair upperLeft, dbpair lowerRight){
            mapElements.push_back(new Obstacle(nextElementID++, upperLeft, lowerRight));
        }
        void addDisk(double xPos, double yPos, char teamColor, int currentCapacity){
            mapElements.push_back(new DiskGroup(nextElementID++, xPos, yPos, teamColor, 2.0, currentCapacity));
        }
        void addGoal(double x, double y, int maxCapacity, char teamColor){
            mapElements.push_back(new FrisbeeGoal(nextElementID++, x, y, teamColor, 6.0, 8.0,maxCapacity));
        }
        void addGoalZone(dbpair upperLeft, dbpair lowerRight, int maxCapacity, char teamColor){
            mapElements.push_back(new GoalZone(nextElementID++, upperLeft, lowerRight, maxCapacity, teamColor));
        }
        void addRoller(dbpair upperLeft, dbpair lowerRight, char color){
            mapElements.push_back(new Roller(nextElementID++, color, upperLeft, lowerRight));
        }
    public:
        std::vector<GameElement*> mapElements;
        /// @brief Map Constructor that Builds the Map
        Map(){
            nextElementID = 0;

            // outer walls
            addObstacle(dbpair(0, 0),     dbpair(140.2, 0));            //0
            addObstacle(dbpair(0, 0),     dbpair(0, 140.2));            //1
            addObstacle(dbpair(0, 140.2), dbpair(140.2, 140.2));        //2
            addObstacle(dbpair(140.4, 0), dbpair(140.4, 140.4));        //3

            // inner walls
            addObstacle(dbpair(92.6, 21.92),   dbpair(92.6, 47.81));    //4
            addObstacle(dbpair(94.6, 21.92),   dbpair(94.6, 47.81));    //5
            addObstacle(dbpair(118.49, 47.81), dbpair(92.6, 47.81));    //6
            addObstacle(dbpair(118.49, 45.81), dbpair(92.6, 45.81));    //7
            addObstacle(dbpair(47.81, 92.6),   dbpair(21.92, 92.6));    //8
            addObstacle(dbpair(47.81, 94.6),   dbpair(21.92, 94.6));    //9
            addObstacle(dbpair(47.81, 94.6),   dbpair(47.81, 118.49));  //10
            addObstacle(dbpair(45.81, 94.6),   dbpair(45.81, 118.49));  //11

            addDisk(72 - 11.30,  11.30 - 72,    'N', 1);                //12    //Idx 0
            addDisk(72 - 23.08,  23.08 - 72,    'N', 1);                //13   //Idx 1
            addDisk(72 - 34.86,  34.86 - 72,    'N', 3);                //14   //Idx 2
            addDisk(72 - 46.64,  46.64 - 72,    'N', 1);                //15   //Idx 3
            addDisk(72 - 58.42,  72.00 - 58.42, 'N', 1);                //16   //Idx 4
            addDisk(72 - 81.99,  81.99 - 72,    'N', 1);                //17   //Idx 5
            addDisk(72 - 93.77,  93.77 - 72,    'N', 1);                //18   //Idx 6
            addDisk(72 - 105.35, 105.35 - 72,   'N', 3);                //19   //Idx 7
            addDisk(72 - 117.33, 117.33 - 72,   'N', 1);                //20   //Idx 8
            addDisk(72 - 129.11, 129.11 - 72,   'N', 1);                //21  //Idx 9
            addDisk(72 - 58.42,  34.86  - 72,   'R', 3);                //22  //Idx 10
            addDisk(72 - 70.20,  146.64 - 72,   'R', 1);                //23  //Idx 11
            addDisk(72 - 81.99,  58.42  - 72,   'R', 1);                //24   //Idx 12
            addDisk(72 - 105.35, 81.99  - 72,   'R', 1);                //25   //Idx 13
            addDisk(72 - 89.83,  26.23  - 72,   'R', 1);                //26   //Idx 14
            addDisk(72 - 89.83,  34.86  - 72,   'R', 1);                //27  //Idx 15
            addDisk(72 - 89.83,  43.89  - 72,   'R', 1);                //28   //Idx 16
            addDisk(72 - 96.52,  50.85  - 72,   'R', 1);                //29   //Idx 17
            addDisk(72 - 105.35, 50.85  - 72,   'R', 1);                //30   //Idx 18
            addDisk(72 - 114.18, 50.85  - 72,   'R', 1);                //31  //Idx 19
            addDisk(72 - 34.86,  58.42  - 72,   'B', 1);                //32  //Idx 20
            addDisk(72 - 58.52,  81.99  - 72,   'B', 1);                //33  //Idx 21
            addDisk(72 - 70.20,  93.77  - 72,   'B', 1);                //34  //Idx 22
            addDisk(72 - 81.99,  105.35 - 72,   'B', 3);                //35   //Idx 23
            addDisk(72 - 26.23,  89.83  - 72,   'B', 1);                //36   //Idx 24
            addDisk(72 - 34.86,  89.83  - 72,   'B', 1);                //37  //Idx 25
            addDisk(72 - 43.89,  89.83  - 72,   'B', 1);                //38   //Idx 26
            addDisk(72 - 50.58,  96.52  - 72,   'B', 1);                //39  //Idx 27
            addDisk(72 - 50.58,  105.35 - 72,   'B', 1);                //40   //Idx 28
            addDisk(72 - 50.58,  114.18 - 72,   'B', 1);                //41   //Idx 29

            // Goals
            addGoal(72 - 122.63, 17.78 - 72, 5, 'B');                   //42
            addGoal(72 - 17.78, 122.63 - 72, 5, 'R');                   //43

            // Rollers need to be offset by 4.9
            //Coordinate at the middle of the roller
            addRoller(dbpair(72 - 29.43, 0.39 - 72),    dbpair(72 - 29.43 , 0.39 - 72),   'N'); //44 //Q3 roller
            addRoller(dbpair(72 - 0.39, 29.43 - 72),    dbpair(72 - 0.39, 29.43 - 72),    'N'); //45
            addRoller(dbpair(72 - 140.02, 110.98 - 72), dbpair(72 - 140.02, 110.98 - 72), 'N'); //46
            addRoller(dbpair(72 - 110.98, 140.02 - 72), dbpair(72 - 110.98, 140.02 - 72), 'N'); //47
        }

        /*
        TODO These two functions will be created after the JSON parser is created.

        void writeMap(){

        }
        void readMap(){

        }
        */



};
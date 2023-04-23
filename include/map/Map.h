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

            addDisk(70.2 - 11.30,  11.30 - 70.2,    'N', 1);                //12    //Idx 0
            addDisk(70.2 - 23.08,  23.08 - 70.2,    'N', 1);                //13   //Idx 1
            addDisk(70.2 - 34.86,  34.86 - 70.2,    'N', 3);                //14   //Idx 2
            addDisk(70.2 - 46.64,  46.64 - 70.2,    'N', 1);                //15   //Idx 3
            addDisk(70.2 - 58.42,  70.2 - 58.42, 'N', 1);                //16   //Idx 4
            addDisk(70.2 - 81.99,  81.99 - 70.2,    'N', 1);                //17   //Idx 5
            addDisk(70.2 - 93.77,  93.77 - 70.2,    'N', 1);                //18   //Idx 6
            addDisk(70.2 - 105.35, 105.35 - 70.2,   'N', 3);                //19   //Idx 7
            addDisk(70.2 - 117.33, 117.33 - 70.2,   'N', 1);                //20   //Idx 8
            addDisk(70.2 - 129.11, 129.11 - 70.2,   'N', 1);                //21  //Idx 9
            addDisk(70.2 - 58.42,  34.86  - 70.2,   'R', 3);                //22  //Idx 10
            addDisk(70.2 - 70.20,  146.64 - 70.2,   'R', 1);                //23  //Idx 11
            addDisk(70.2 - 81.99,  58.42  - 70.2,   'R', 1);                //24   //Idx 12
            addDisk(70.2 - 105.35, 81.99  - 70.2,   'R', 1);                //25   //Idx 13
            addDisk(70.2 - 89.83,  26.23  - 70.2,   'R', 1);                //26   //Idx 14
            addDisk(70.2 - 89.83,  34.86  - 70.2,   'R', 1);                //27  //Idx 15
            addDisk(70.2 - 89.83,  43.89  - 70.2,   'R', 1);                //28   //Idx 16
            addDisk(70.2 - 96.52,  50.85  - 70.2,   'R', 1);                //29   //Idx 17
            addDisk(70.2 - 105.35, 50.85  - 70.2,   'R', 1);                //30   //Idx 18
            addDisk(70.2 - 114.18, 50.85  - 70.2,   'R', 1);                //31  //Idx 19
            addDisk(70.2 - 34.86,  58.42  - 70.2,   'B', 1);                //32  //Idx 20
            addDisk(70.2 - 58.52,  81.99  - 70.2,   'B', 1);                //33  //Idx 21
            addDisk(70.2 - 70.20,  93.77  - 70.2,   'B', 1);                //34  //Idx 22
            addDisk(70.2 - 81.99,  105.35 - 70.2,   'B', 3);                //35   //Idx 23
            addDisk(70.2 - 26.23,  89.83  - 70.2,   'B', 1);                //36   //Idx 24
            addDisk(70.2 - 34.86,  89.83  - 70.2,   'B', 1);                //37  //Idx 25
            addDisk(70.2 - 43.89,  89.83  - 70.2,   'B', 1);                //38   //Idx 26
            addDisk(70.2 - 50.58,  96.52  - 70.2,   'B', 1);                //39  //Idx 27
            addDisk(70.2 - 50.58,  105.35 - 70.2,   'B', 1);                //40   //Idx 28
            addDisk(70.2 - 50.58,  114.18 - 70.2,   'B', 1);                //41   //Idx 29

            // Goals
            addGoal(70.2 - 122.63, 17.78 - 70.2, 5, 'B');                   //42
            addGoal(70.2 - 17.78, 122.63 - 70.2, 5, 'R');                   //43

            // Rollers need to be offset by 4.9
            //Coordinate at the middle of the roller
            addRoller(dbpair(70.2 - 29.43, 0.39 - 70.2),    dbpair(70.2 - 29.43 , 0.39 - 70.2),   'N'); //44 //Q4 roller
            addRoller(dbpair(70.2 - 0.39, 29.43 - 70.2),    dbpair(70.2 - 0.39, 29.43 - 70.2),    'N'); //45
            addRoller(dbpair(70.2 - 140.02, 110.98 - 70.2), dbpair(70.2 - 140.02, 110.98 - 70.2), 'N'); //46
            addRoller(dbpair(70.2 - 118.49, 140.02 - 70.2), dbpair(70.2 - 106.6, 140.02 - 70.2), 'N'); //47 Q2?
        }

        /*
        TODO These two functions will be created after the JSON parser is created.

        void writeMap(){

        }
        void readMap(){

        }
        */



};
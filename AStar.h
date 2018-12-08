//
// Created by Po-Teng on 2018/11/25.
//

#ifndef CLION_ASTARSEARCH_H
#define CLION_ASTARSEARCH_H

#include<bits/stdc++.h>
using namespace std;

#define watch(x) cout << (#x) << " is " << (x) << endl

struct posNode{
    long x;
    long y;
    double f;
    double g;
    const posNode *prev = nullptr;
    posNode(long xx, long yy, double gg) : x(xx), y(yy), g(gg), f(-1) {};
    posNode() : f(-1) {};

    bool operator == (const posNode& node) const {
        return (node.x == x && node.y == y);
    }
};

struct posNodeHash{
    size_t operator()(const posNode& node) const {
        return hash<long>()(node.x + node.y);
    }
};

enum heuristicType{Diagonal, Manhattan, Euclidean};

enum constraintType{Wall, Height, None};

class AStar {

private:
    unsigned long lengthMap = 0;        //size of the map
    unsigned long widthMap = 0;
    long highMap = 0;                   //highest height of the map
    long lowMap = 0;                    //lowest height of the map
    long steep = 10;                    //how likely the obstacles will be generated
    heuristicType hType = Euclidean;    //which heuristic function to use
    constraintType cType = Height;      //if set to Wall, all terrain height except for 0 will be treated as obstacles
    const char * filename = "map.txt";  //filename of the map
    fstream file;
    vector<vector<int>> vecMapInfo;     //vector containing map information
    map<pair<long, long>, vector<double>> movingObs;    //map to record the occupation of other moving objects
                                                        // [(x, y) , time]   the time is posNode.g at here
    vector<pair<pair<long, long>, long>> path;                      //vector to record the found path


    bool openMapInfoRead(const char *name);
    bool openMapInfoWrite(const char *name);

    void showPathWithMap(vector<vector<string>>&);

    //any two continue points on the line should not have difference more than 1 in both direction
    //need slope < 1 to avoid gap
    bool constraintObject(long xNew, long yNew, double time);

    bool constraintMap(long xOld, long yOld, long xNew, long yNew);

    double heuristicFunction(long xOri, long yOri, long xDes, long yDes);
    double diagonalDistance(long xOri, long yOri, long xDes, long yDes);
    double manhattanDistance(long xOri, long yOri, long xDes, long yDes);
    double euclideanDistance(long xOri, long yOri, long xDes, long yDes);

    void clearPath();

public:
    //constructor initialize 5 members. 1-2 size of map 3-4 height of map 5 steep
    //steap 10: only 10% of grids might not be 0
    //steep 100: 100% of grids might not be 0
    AStar(unsigned long lengthMap, unsigned long widthMap, long highMap, long lowMap, long steep);

    //get and set filename for map information
    const char *getFilename() const;
    void setFilename(const char *filename);

    //get and set the type of heuristic function
    heuristicType getHType() const;
    void setHType(heuristicType hType);

    //get and set the type of constraint
    constraintType getCType() const;
    void setCType(constraintType cType);

    //create a random map
    bool createMapInfo();

    //read map info from file
    bool extractMapInfo();

    //print the map info
    void showMapInfo();

    //draw found path. this function return length of the path
    long drawLine();

    //draw found path. also draw the obstacles blocking in the way
    pair<long, long> drawLineObstacle();

    //print path points
    void showPath();

    //get the positions of origin and destination, and store found path
    bool aStarSearchMoving(unsigned long xOri, unsigned long yOri, unsigned long xDes, unsigned long yDes);
};


#endif //CLION_ASTARSEARCH_H

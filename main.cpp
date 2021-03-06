//
// Created by Po-Teng on 2018/11/25.
//
/* Learned: when putting objects into container, the copies are put.
 * It's possible to just use object/struct instead of their pointers.
 * To get the pointer of an object from a container, use find() to get iterator, * to get object, & to get its address:
 * ptr = &*set.find(obj);
 * http://www0.cs.ucl.ac.uk/staff/d.silver/web/Publications_files/coop-path-AIWisdom.pdf
 */

#include<bits/stdc++.h>
#include"AStar.h"

using namespace std;

#define watch(x) cout << (#x) << " is " << (x) << endl


int main() {

    string input;

    AStar road(50, 60, 10, -2, 50);
    road.setHType(Euclidean);
    road.setCType(None);

    cout << "create new map?" <<endl;
    cin >> input;
    if (input == "y"){
        road.createMapInfo();
    }

    //get mapInfo from file to vectors
    if ( road.extractMapInfo() ){
        road.showMapInfo();
    }


    //If a path from (5, 47) to (38, 3) can be found, draw the path
    if (road.aStarSearchMoving(5, 47, 38, 3)){
        road.drawLineObstacle();
    }
    //If a path from (11, 9) to (43, 47) can be found, print out the path coordinates and draw the path
    if (road.aStarSearchMoving(11, 9, 43, 47)){
        road.showPath();
        road.drawLineObstacle();
    }
    if (road.aStarSearchMoving(19, 28, 49, 52)){
        road.drawLineObstacle();
    }

    cout << "Finished." << endl;
    return 0;
}

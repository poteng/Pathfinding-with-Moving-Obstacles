//
// Created by Po-Teng on 2018/11/25.
//
#include<bits/stdc++.h>
#include "AStar.h"
using namespace std;

bool AStar::openMapInfoRead(const char *name) {
    file.open(name, ios::in);
    if (!file)
    {
        cout << "\"" << name << "\" (for read) does not exist." << endl;
        return false;
    }
    return true;
}

bool AStar::openMapInfoWrite(const char *name) {
    file.open(name, ios::out);
    if (!file)
    {
        cout << "\"" << name << "\" (for write) does not exist." << endl;
        return false;
    }
    return true;
}
//createMapInfo(lengthMap, widthMap, highMap, lowMap, steep);
bool AStar::createMapInfo() {
    if (!openMapInfoWrite(filename)) return false;

    long diff = highMap - lowMap;
    if (diff == 0) diff = 1;
    steep %= 100;

    for (int i = 0; i < lengthMap; i++) {
        for (int j = 0; j < widthMap; j++) {
            long height = rand() % 100 < steep ? rand() % (diff + 1) + lowMap : 0;

            file << height << " ";
        }
        file << endl;
    }
    file.close();
    return true;
}

bool AStar::extractMapInfo() {
    if (!openMapInfoRead(filename)){
        return false;
    }
    for (auto& vec:vecMapInfo){
        for (int& i:vec){
            file >> i;
        }
    }
    return true;
}

bool AStar::constraintObject(long xNew, long yNew, double time) {
    bool valid = true;
    for (double d:movingObs[{xNew, yNew}]){
        if (d == time){
            valid = false;
        }
    }
    return valid;
}

void AStar::showMapInfo() {
    for (auto& vec:vecMapInfo){
        for (int i:vec){
            cout << i << " ";
        }
        cout << endl;
    }
}

void AStar::showPathWithMap(vector<vector<string>>& vecMapInfo) {
    int i = 0, j = 0;
    cout << endl;
    for (auto& vec:vecMapInfo[0]){
        cout << (j++)%10 << " ";
    }
    cout << endl;
    for (auto& vec:vecMapInfo){
        for (const string &str:vec){
            cout << str << " ";
        }
        cout << i++ <<endl;
    }
}

bool AStar::constraintMap(long xOld, long yOld, long xNew, long yNew) {
    if (cType == Wall){
        return vecMapInfo[xNew][yNew] == 0;
    } else if (cType == Height){
        return abs( vecMapInfo[xNew][yNew] - vecMapInfo[xOld][yOld] ) <= 1;
    } else if (cType == None) {
        return true;
    } else {
        return true;
    }
}

long AStar::drawLine() {
    vector<vector<string>> drawMap(lengthMap, vector<string>(widthMap, " "));
    long lenPath = path.size();
    long timePath = path.front().second;
    if (drawMap.empty() || drawMap[0].empty() || path.empty()) return -1;

    for (auto& p:path){
        drawMap[p.first.first][p.first.second] = 'o';
        lenPath++;
    }

    drawMap[path.front().first.first][path.front().first.second] = 'D';
    drawMap[path.back().first.first][path.back().first.second] = 'O';
    showPathWithMap(drawMap);
    cout << "Length of path: " << lenPath << "\nTime of path: " << timePath << endl;
    return lenPath;
}

pair<long, long> AStar::drawLineObstacle() {
    vector<vector<string>> drawMap(lengthMap, vector<string>(widthMap, " "));
    if (drawMap.empty() || drawMap[0].empty() || path.empty()) return {-1, -1};
    long lenPath = path.size();
    long timePath = path.front().second;
            watch("drawLineObstacle");
    watch(path.size());
    for (auto& p:path){
        drawMap[p.first.first][p.first.second] = 'o';
        for (int k = 0; k < 9; k++){
            long i = p.first.first + k % 3 - 1;
            long j = p.first.second + k / 3 - 1;
            if (i >= 0 && i < drawMap.size() && j >= 0 && j < drawMap[0].size()){
                if (!constraintMap(p.first.first, p.first.second, i, j)){
                    drawMap[i][j] = "X";
                }
            }
        }
    }

    drawMap[path.front().first.first][path.front().first.second] = 'D';
    drawMap[path.back().first.first][path.back().first.second] = 'O';
    showPathWithMap(drawMap);
    cout << "Length of path: " << lenPath << "\nTime of path: " << timePath << endl;
    return {lenPath, timePath};
}

double AStar::diagonalDistance(long xOri, long yOri, long xDes, long yDes) {
    return max( abs(xDes - xOri), abs(yDes - yOri) );
}

double AStar::manhattanDistance(long xOri, long yOri, long xDes, long yDes) {
    return abs(xDes - xOri) + abs(yDes - yOri);
}

double AStar::euclideanDistance(long xOri, long yOri, long xDes, long yDes) {
    return sqrt( (xDes - xOri) * (xDes - xOri) + (yDes - yOri) * (yDes - yOri) );
}

double AStar::heuristicFunction(long xOri, long yOri, long xDes, long yDes) {
    if (hType == Diagonal){
        return diagonalDistance(xOri, yOri, xDes, yDes);
    } else if (hType == Manhattan){
        return manhattanDistance(xOri, yOri, xDes, yDes);
    } else if (hType == Euclidean){
        return euclideanDistance(xOri, yOri, xDes, yDes);
    }
}

bool AStar::aStarSearchMoving(unsigned long xOri, unsigned long yOri, unsigned long xDes, unsigned long yDes) {
    unordered_set<posNode, posNodeHash> openSet, closeSet;
    long xNew, yNew;
    bool found = false;

    //struct for posNode:   x, y, f, g, prev
    //put origin in open set.   set g longer if there is multiple object in the origin
    posNode origin(xOri, yOri, 0);    //g = 0;
    while (!constraintObject(xOri, yOri, origin.g)){
        origin.g++;
    }
    posNode minNode;
    origin.f = origin.g + heuristicFunction(xOri, yOri, xDes, yDes);
    openSet.insert(origin);

    //for each pos in open set
    while (!openSet.empty()){
        minNode.f = -1;         //clear minNode at beginning
        //pop a smallest pos into close set
        for (auto& node:openSet){
            if (minNode.f == -1){
                minNode = node;
            } else {
                if (node.f < minNode.f){
                    minNode = node;
                }
            }
        }
        if (minNode.x == xDes && minNode.y == yDes){    //reached goal
            found = true;
            break;
        }

        openSet.erase(minNode);
        closeSet.insert(minNode);

        //put its 8 directions pos (not in close set) in stack. give them info about g = pos.dis + 1, h = diagonal distance
        for (int i = 0; i < 9; i++){
            xNew = minNode.x + i % 3 - 1;
            yNew = minNode.y + i / 3 - 1;
            if (xNew < 0 || xNew >= vecMapInfo.size() || yNew < 0 || yNew >= vecMapInfo[0].size()){ //boundary
                continue;
            }
            if ( !constraintMap(minNode.x, minNode.y, xNew, yNew)){             //obstacles/height
                continue;
            }

            posNode childNode( xNew, yNew, minNode.g + 1);
            if (closeSet.count(childNode) == 0){
                childNode.f = childNode.g + heuristicFunction(xNew, yNew, xDes, yDes);
                //check moving object and wait
                while (!constraintObject(xNew, yNew, childNode.g)){
                    cout << "encounter moving obstable at " << xNew << ", " << yNew << " ,time "
                         << childNode.g << ", wait until time " << childNode.g+1 << endl;
                    childNode.g++;
                }
                childNode.prev = &*closeSet.find(minNode);      //get the pointer in the set
                openSet.insert(childNode);
            }
        }
    }

    //clear path every time before having a new path
    clearPath();

    int countPath = 1;
    if (found){
        //build the whole path through ->prev member (from Des to Ori)
        //use find to find the iterator pointing to object in the set, and the the reference of it
        auto curNode = &*openSet.find(minNode);
        while (curNode && curNode->x != xOri && curNode->y != yOri){
            //occupy the position at corresponding timing, which is posNode->g
            movingObs[{curNode->x, curNode->y}].emplace_back(curNode->g);

            path.emplace_back(make_pair(make_pair(curNode->x, curNode->y), curNode->g));
            //cout << countPath++ << ".\t("<< path.back().first.first <<", "<< path.back().first.second <<") "<<path.back().second<<endl;
            curNode = curNode->prev;
        }
        path.emplace_back(make_pair(make_pair(curNode->x, curNode->y), curNode->g));
        //cout << countPath << ".\t("<< path.back().first.first <<", "<< path.back().first.second <<") "<<path.back().second<<endl;
    }
    return found;
}

AStar::AStar(unsigned long lengthMap, unsigned long widthMap, long highMap, long lowMap, long steep) : lengthMap(
        lengthMap), widthMap(widthMap), highMap(highMap), lowMap(lowMap), steep(steep) {
    vecMapInfo = vector<vector<int>>(lengthMap, vector<int>(widthMap));
    srand(static_cast<unsigned int>(time(nullptr)));
}

void AStar::clearPath() {
    path.clear();
}

void AStar::showPath() {
    int countPath = 1;
    cout << "showPath():" << endl;
    for (auto i:path){
        cout<<countPath<<".\t("<<i.first.first<<", "<<i.first.second<<") "<<i.second<<endl;
        countPath++;
    }
}

heuristicType AStar::getHType() const {
    return hType;
}

void AStar::setHType(heuristicType hType) {
    AStar::hType = hType;
}

constraintType AStar::getCType() const {
    return cType;
}

void AStar::setCType(constraintType cType) {
    AStar::cType = cType;
}

const char *AStar::getFilename() const {
    return filename;
}

void AStar::setFilename(const char *filename) {
    AStar::filename = filename;
}







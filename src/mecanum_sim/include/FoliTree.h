#ifndef _FOLITREE_H_
#define _FOLITREE_H_
#include "ros/ros.h"
#include <iostream>
#include <PositionAll.h>
#include <JacobianPoint.h>
#include <FoliTreeNode.h>
#include <time.h>
using namespace std;
class FoliTree{
public:
    FoliTree(){}
    FoliTree(const FoliTree& another);
    FoliTree(FoliTreeNode* ancestry_);
    ~FoliTree(){}
    void getShortestPath();
    FoliTreeNode* ancestry;
    FoliTreeNode* nearestgrandson;
    vector<FoliTreeNode*> outermost;
    vector<FoliTreeNode*> subouter;
    vector<FoliTreeNode*> shortestPath;
};
#endif

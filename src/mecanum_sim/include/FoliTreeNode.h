#ifndef _FOLITREENODE_H_
#define _FOLITREENODE_H_
#include "ros/ros.h"
#include <iostream>
#include <PositionAll.h>
#include <JacobianPoint.h>
#include <time.h>
using namespace std;

class FoliTreeNode{
public:
    FoliTreeNode(){}
    FoliTreeNode(double th1_, double th2_, double th3_, double th4_, double th5_, double th6_, double x0_, double y0_, double alph_, int level_);
    FoliTreeNode(const FoliTreeNode& anotherFoli);
    ~FoliTreeNode(){}
    double CalcDistToOther(FoliTreeNode*);
    double th1, th2, th3, th4, th5, th6, x0, y0, alph;
    double dw1, dw2, dw3, dw4, dw5, dw6, dw7, dw8, dw9;
    int level;
    double dist_to_init;
    double dist_to_obstacle;
    int clearest_point_index;
    FoliTreeNode* Father;
    vector<FoliTreeNode*> Sons;
};
#endif


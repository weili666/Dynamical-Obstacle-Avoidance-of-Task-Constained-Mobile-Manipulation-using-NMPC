#include <FoliTree.h>

using namespace std;

FoliTree::FoliTree(const FoliTree& another)
{
    ancestry = another.ancestry;
    nearestgrandson = another.nearestgrandson;
    outermost = another.outermost;
    subouter = another.subouter;
    shortestPath = another.shortestPath;
}

FoliTree::FoliTree(FoliTreeNode* ancestry_)
{
    ancestry = ancestry_;
}

void FoliTree::getShortestPath()
{
    vector<FoliTreeNode*> inversePath;
    FoliTreeNode* Handle_now = nearestgrandson;
    do{
        inversePath.push_back(Handle_now);
        Handle_now = Handle_now->Father;
    }
    while(Handle_now != ancestry);
    for(int i = inversePath.size()-1; i >= 0; i --)
    {
        shortestPath.push_back(inversePath[i]);
    }
}

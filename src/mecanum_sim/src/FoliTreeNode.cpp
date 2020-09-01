#include <FoliTreeNode.h>
using namespace std;

FoliTreeNode::FoliTreeNode(double th1_, double th2_, double th3_, double th4_, double th5_, double th6_, double x0_, double y0_, double alph_, int level_)
{
    th1 = th1_;
    th2 = th2_;
    th3 = th3_;
    th4 = th4_;
    th5 = th5_;
    th6 = th6_;
    x0 = x0_;
    y0 = y0_;
    alph = alph_;
    level = level_;
}

FoliTreeNode::FoliTreeNode(const FoliTreeNode& anotherFoli)
{
    th1 = anotherFoli.th1;
    th2 = anotherFoli.th2;
    th3 = anotherFoli.th3;
    th4 = anotherFoli.th4;
    th5 = anotherFoli.th5;
    th6 = anotherFoli.th6;
    x0 = anotherFoli.x0;
    y0 = anotherFoli.y0;
    alph = anotherFoli.alph;
    level = anotherFoli.level;
}

double FoliTreeNode::CalcDistToOther(FoliTreeNode* another)
{
    double w1, w2, w3, w4, w5, w6, w7, w8, w9;
    w1 = 1.0; w2 = 1.0; w3 = 1.0; w4 = 1.0; w5 = 1.0; w6 = 1.0; w7 = 1.0; w8 = 1.0; w9 = 1.0;
    double d = sqrt(w1*(th1-another->th1)*(th1-another->th1)+w2*(th2-another->th2)*(th2-another->th2)
                    +w3*(th3-another->th3)*(th3-another->th3)+w4*(th4-another->th4)*(th4-another->th4)
                    +w5*(th5-another->th5)*(th5-another->th5)+w6*(th6-another->th6)*(th6-another->th6)
                    +w7*(x0-another->x0)*(x0-another->x0)+w8*(y0-another->y0)*(y0-another->y0)
                    +w9*(alph-another->alph)*(alph-another->alph));
    return d;
}

#ifndef __GA_H
#define __GA_H

#include "route.h"

#define MAX_GEN   1000

void mutate(vector <PathsVector> &vect,Graph *G,bool limit);
void exchange_two_demand_points(vector <PathsVector> &GlobalVec,Graph *G,int r1,int r2,bool limit);
void ga(vector <PathsVector> &vect, vector <vector <PathsVector> > &pop,Graph *G);

#endif






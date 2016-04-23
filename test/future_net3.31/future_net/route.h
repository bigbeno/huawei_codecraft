#ifndef __ROUTE_H__
#define __ROUTE_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <stack>
#include <list> 
#include <sstream>

using namespace std;
#define MAX_VERTEX_NUM 600
#define VERTEX_OFFSET  1
#define NIL           -1

typedef struct arcnode
{	   int arc;
  	   int vertex; //��ͷ���ٽڵ� 
 	   int weight;
 	   arcnode *next;
}arcnode;

typedef struct vernode 
{
 		int vex; //�����
		arcnode *next; 
} vertex;

typedef struct  
{
 	vertex v[MAX_VERTEX_NUM];
	int numofVex;  //������� 
	int numofEdge; //���ĸ��� 
}Graph;

typedef struct 
{
 int vertex;
 int weight;
}NodeInfo;

typedef struct 
{
 int srcvex; //ÿһ��·�ĳ����� 
 int dstvex; //ÿһ���ڵ��Ŀ�Ľڵ� 
 int weight; //һ��·����Ȩ�� 
 std::vector <NodeInfo> node;
}DemandVexInfo;

typedef struct 
{
 int srcvex;
 int dstvex;
 int len;   //�ؾ�����ܳ��� 
 std::vector <int> demand;//��src��Ҳ�ӽ�ȥ����Ϊ��һ��������㲻�ܾ������� demand[0] 
}DemandVertex;


void search_route(char *topo[5000], int edge_num, char *condition);
void graph_init(Graph *G,int edge_num);
void build_graph(Graph *G,char *topo[5000],char *demand,DemandVertex &Vertex);
void destory_graph(Graph *G);
void print_graph(Graph *G);
void find_closest_points(Graph *G,string &outStr);

#endif

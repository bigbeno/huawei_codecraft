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
#include <ctime>
#include <math.h>

using namespace std;
#define MAX_VERTEX_NUM	   600
#define VERTEX_OUT_OFFSET  1
#define VERTEX_IN_OFFSET   2
#define VERTEX_W_OFFSET    3
#define NIL           	  -1

typedef struct arcnode
{	   int arc;
  	   int vertex; //��ͷ���ٽڵ� 
  	   int prev_vex;//prev vertex id number
 	   int weight;
 	   arcnode *next;
 	   arcnode *prev;//prev arcnode to solve bridge 
}arcnode;

typedef struct vernode 
{
 		int vex; //�����
 		int indegree;
 		int outdegree;
		arcnode *next; 
		arcnode *prev;
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
 double weight; //һ��·����Ȩ�� 
 double pheromone; //��Ϣ��
 std::vector <NodeInfo> node;
}DemandVexInfo;

typedef struct 
{
 int srcvex;
 int dstvex;
 int len;   //�ؾ�����ܳ��� 
 std::vector <int> demand;//��src��Ҳ�ӽ�ȥ��Ϊ��һ�� demand[0]���յ���dstvex 
}DemandVertex;

typedef struct pathvector
{
	int src;
	int dst;    // next destination
	int weight;
	vector <int> path;
}PathsVector;
//optimization using dfs
void search_route(char *topo[5000], int edge_num, char *condition);
void graph_init(Graph *G,int edge_num);
void build_graph(Graph *G,char *topo[5000],char *demand,DemandVertex &Vertex);
void destory_graph(Graph *G);
void print_graph(Graph *G);
void find_closest_points(Graph *G,string &outStr);
bool connected_in_map(int src,int dst);
void dijkstra(Graph *G,int src,int d[],int p[],int done[], map<int,int> mapInfo);
void record_route(int src,int dst,int path[],vector <NodeInfo> &Info,Graph *G);

#endif

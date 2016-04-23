#include "route.h"
#include "lib_record.h"
#include "multiway_tree.h"
#include "lib_time.h"
#include "ga.h"

#define _INFINITY  10000000
#define SET    1
#define RESET  0
#define MAX_DEMAND_VERTEX 100
#define MAX_VERTEX_NUM    600 
#define EDGE_OF_DEMAND_VERTEX	6
#define DemandVexSize   (demandVex.demand.size())
#define OUT_MINISEC          	7
#define OUT_STRATEGY_ONE      	2

#define MAX_OUTTIME_LIMIT      (12.2)

#define TIME_SLICE        	1 

#define DFS_LOOP_TIMEOUT  600

#define FORWARD        	  0     	//bfs search direct 

#define BACKWARD       	  1     	//bfs search direct

#define STRATEGY_ONE      0x10
#define STRATEGY_TWO      0x11

// status
// 3.0 can't solve 14
// 2.0 solve all
// 1.0 can't solve 14
#define PATH_LENGTH_WEIGHT  	(average_weight * (double) 2.0)

#define ENABLE_DIJKSTRA

#define DEMAND_VERTEX_IN_EACH_PATH     4

//#define USING_STRATEGY_FIRST 	       1
//#define USING_STRATEGY_SECOND        1
#define USING_STRATEGY_BOTH            1
#define INIT_SEED_CLOCKS               1

//using 200 has a good result
// ant algorithm

const double ALPHA = 1.0;
const double BETA  = 2.0;
const double ROU   = 0.8;     //reserve， evaporate (1-ROU)
const double Pbest = 0.05;   //possibily to find the best route in one search 
const double LUO   = 0.3;

bool BestOfAll = false; 

double cityNum;
double dbTemp;
double m_dbRate;
double ant_max;
double ant_min;

static int weight_t = _INFINITY;
static unsigned int milisec;

unsigned int time_limit;
int  DynamicChangeNode = 6; //change according Graph info  
DemandVertex demandVex;
vector <int> LastConnectVertex;

vector <vector<double> > roulette_prob; 
int mutate_cnt;
//int nowstep;

#define TABUSTEP   2
int  TABUSIZE  = 5;

typedef struct
{ 
   int depth;
   int vertex;	
} TabuStruct;

vector<TabuStruct> TabuTab;

int visit[MAX_DEMAND_VERTEX][MAX_VERTEX_NUM];
int _visit[MAX_DEMAND_VERTEX][MAX_VERTEX_NUM];

int closest[MAX_DEMAND_VERTEX][20];
TreeNode treeNode[MAX_DEMAND_VERTEX];
TreeNode _treeNode[MAX_DEMAND_VERTEX];
vector<vector<DemandVexInfo> >   pathInfo;
vector<vector<DemandVexInfo> >  _pathInfo;			//very inportant contain all the infomation of the route 
map <int,vector<DemandVexInfo> > map2Vector;  		//should be initial at the begining 
int randmomMap[1000][MAX_DEMAND_VERTEX] = {0};
double  average_weight = 0;
int connected_map[MAX_VERTEX_NUM][MAX_VERTEX_NUM]; 	//to restore information between two demanded vertexs

map <int,int>  demandVertexIndexMap;
// dijkstra needed 
int AllVexterNum [1000];
int Dis_dijkstra [1000];
int Path_dijkstra[1000];
int Done_dijktra [1000]; 
int ShortesPath[MAX_DEMAND_VERTEX][100]; 			//ShortestPath[x][0] store the length of each route

map <int,int> OutDegreeOne;
map <int,int> InDegreeOne;
map <int,int> InDegreeCnt;

int  get_edge_number(Graph *G,int vstart,int vend);
void refresh_write_buff(string outStr);
vector <int> generate_possible_route(vector<int> vertex,vector<int> &vertex_new,int endVertex,vector <NodeInfo> &Path);
int DFS(map <int,int> &Map,int startVertex ,int &VertexNum ,list<int> &pathList,list<int> &FinalList, int miniweight,Graph *G,int flag,bool &stagnation_flag);
int main_loop(int startVertex,list<int> &FinalList,Graph *G);
int get_edge_weight(Graph *G,int vstart,int vend);
void map_init(vector <vector<DemandVexInfo> > pathInfo,map <int,vector<DemandVexInfo> >  &map2Vector ,DemandVertex demand);
bool pass_all_demand_vertex(map <int,int> &mapInfo ,int vex);
void shortest_path_from_A2B(int src,int dst,map <int,int> &mapInfo,vector <int> &vec,Graph *G,int &weight,bool global);
static void format_path_info(list<int> &pathList,Graph *G,vector <PathsVector> &vect);
static void update_roulette(void);
static void evaporate_pheromone(void);
static void ant_sys_stagnation_handler(void);
void search_route(char *topo[5000], int edge_num, char *demand)
{
    
}

void get_demand_vertex(char *demand,DemandVertex &Vertex)
{
	const char *delim = ",|";
	char *s = NULL;
	s = strtok(demand,delim);
	Vertex.srcvex = atoi(s);
	Vertex.demand.push_back(Vertex.srcvex);  
	s = strtok(NULL,delim);
	Vertex.dstvex = atoi(s);
	s = strtok(NULL,delim);
	while (s != NULL)
	{
		Vertex.demand.push_back(atoi(s));
		s = strtok(NULL,delim);
	}	 
}

static inline arcnode* assign_node(int nodeInfo[])
{
  arcnode *ptr = NULL;
  ptr = ( arcnode *)malloc(sizeof(arcnode));
  if(ptr == NULL) return NULL; 
  ptr->arc  	= nodeInfo[0];
  ptr->prev_vex = nodeInfo[1];
  ptr->vertex 	= nodeInfo[2];
  ptr->weight   = nodeInfo[3]; 
  ptr->next 	= NULL;
  ptr->prev     = NULL;
  return ptr;
}

void graph_init(Graph *G,int edge_num)
{
 	 int i ,j;
 	 for(i = 0;i < MAX_VERTEX_NUM;i++)
 	 {
	  	   G->v[i].vex  = NIL;
	  	   G->v[i].next = NULL;
	  	   G->v[i].prev = NULL;
	  	   G->v[i].indegree = 0;
	  	   G->v[i].outdegree = 0;
   	 }
   	 G->numofEdge = edge_num;
   	 G->numofVex  = 0;
}

void build_graph(Graph *G,char *topo[5000],char *demand,DemandVertex &Vertex)
{
 	 const char *delim = ",";
 	 char *s = NULL;
 	 int   nodeInfo[10],offset;
 	 vertex *v; 
 	 int i;
 	 int cnt = 0;
 	 for(i = 0; i < G->numofEdge; i++)
 	 {
	  	   s 	  = strtok(topo[i],delim);
	  	   offset = 0;
	  	   while(s != NULL )
	  	   {
		   		 nodeInfo[offset++] = atoi(s);
		   		 s = strtok(NULL,delim);
  		   }
  		   // update outdegree info of node
	  	    v = &G->v[nodeInfo[VERTEX_OUT_OFFSET]]; 
	  	    
	  	    if(NULL == v->next && NULL == v->prev) G->numofVex++;
	  	    
			if(v->next != NULL) //add outdegree 
			{	
				arcnode *arc = v->next;
				while(arc->next != NULL){arc = arc->next;}
				arc->next = assign_node(nodeInfo);	
				v->outdegree++;              				   
			} else {
   			    v->vex 	= nodeInfo[VERTEX_OUT_OFFSET];
			  	v->next = assign_node(nodeInfo);  
			  	v->outdegree++;
            }
            
			// update indegree info of node
			v = &G->v[nodeInfo[VERTEX_IN_OFFSET]]; 
	  	    if(NULL == v->next && NULL == v->prev) G->numofVex++;
	  	    
			if(v->prev != NULL) //add outdegree 
			{	
				arcnode *arc = v->prev;
				while(arc->prev != NULL){arc = arc->prev;}
				arc->prev = assign_node(nodeInfo);
				v->indegree++;	              				   
			} else {
   			    v->vex 	= nodeInfo[VERTEX_IN_OFFSET];
			  	v->prev = assign_node(nodeInfo);
				v->indegree++;  
            }
        	//to caculate the total weight value
			average_weight+= nodeInfo[VERTEX_W_OFFSET]; 				  	     	   
	 }	 
   
	 get_demand_vertex(demand,Vertex); 
	 average_weight = average_weight / G->numofEdge;  //caculated average weight
}

void destory_graph(Graph *G)
{
	int i;
	arcnode *next,*prev,*ptr; 
	for(i = 0;i < MAX_VERTEX_NUM;i++)
	{	   
		next = G->v[i].next;
		prev = G->v[i].prev;
		while (next != NULL)
		{	ptr = next;
			next = next->next;  
			free(ptr);
		}
		
		while(prev != NULL)
		{
			ptr = prev;
			prev = prev->prev;  
			free(ptr);
		}
	} 	 
}

void print_graph(Graph *G)
{
 	 int i;
	 arcnode *node;
 	 for(i = 0;i < MAX_VERTEX_NUM;i++)
 	 {
  	    if (G->v[i].vex == -1) continue;
	    node = G->v[i].next;
  	    while(node != NULL)
  	    {    
	   		node = node->next;  
	    }
     } 
}

#define VSRC    0
#define VDST    1
#define VOTHER  2

int is_start_or_end_vertex(int vertex)
{
  if(vertex == demandVex.dstvex )  return VDST;
  else if (vertex == demandVex.srcvex) return VSRC;
  return VOTHER;	
}

bool is_demand_vertex(int vertex)
{
 	 if(vertex == demandVex.dstvex) return true;
 	 vector<int>::iterator iter = find(demandVex.demand.begin(),demandVex.demand.end(),vertex);
 	 if (iter == demandVex.demand.end())  return false;
 	 return true;
}

void init_tree_node(TreeNode &node,int vertex)
{
 	 node.vertex = vertex;
 	 node.weight = 0;
}

/*find N closed points among demand points*/

void _BFS(Graph *G,int i,int visit[],TreeNode *tree)
{
 	 arcnode *node ;
 	 TreeNode *tnode;
 	 queue<int> q;
 	 map<int,int> Map;
 	 int count = 0;
 	 visit[i] = 1;     
 	 q.push(G->v[i].vex); 
 	 while( !q.empty())
	 {
	     int k = q.front();
		 q.pop();
         node  = G->v[k].next;
         tnode = find_node(tree,k);
	 	 while(NULL != node)
	 	 {
 			    if(visit[node->vertex] == 0)       //if haven't visit the point
 			    {
				 	visit[node->vertex] = 1;
				 	TreeNode *child = new TreeNode();
					child->vertex = node->vertex;
					child->weight = node->weight;
					tnode->child.push_back(child); //added to the parent's children 
					if (VOTHER == is_start_or_end_vertex(node->vertex))
					{
						q.push(node->vertex);
					}								  
  				}
  				node = node->next;	
		 }		 	
  	 } 
}

void _get_path_info(TreeNode *tnode,vector <NodeInfo> &path,vector<DemandVexInfo> &PathInfo,int demand_cnt)
{
    vector<TreeNode*>::iterator iter  = tnode->child.begin();
    while(iter < tnode->child.end())
    {
	    NodeInfo info;
	    info.vertex = (*iter)->vertex;
	    info.weight = (*iter)->weight;
        path.push_back(info);
        if( VOTHER == is_start_or_end_vertex(info.vertex))
		{
			if((*iter)->child.empty()) // have no children
	        {
	        	if(true == is_demand_vertex(info.vertex)) //take this case into consideration  
	        	{
	        		DemandVexInfo vexpath;		 	   
					vexpath.node = path;    		
			 		PathInfo.push_back(vexpath);
				}
			
			} else if(true == is_demand_vertex(info.vertex))
			{
				if(demand_cnt <= DEMAND_VERTEX_IN_EACH_PATH)
				{
					demand_cnt++;
					DemandVexInfo vexpath;		 	   
					vexpath.node = path;    		
				 	PathInfo.push_back(vexpath);	
					_get_path_info(*iter,path,PathInfo,demand_cnt);
					demand_cnt--;
				}
						
			} else 
			{
				_get_path_info(*iter,path,PathInfo,demand_cnt);
			}	
		} else if ( VDST == is_start_or_end_vertex(info.vertex)) 
		{
				DemandVexInfo vexpath;		 	   
				vexpath.node = path;   		
			 	PathInfo.push_back(vexpath);	
		}      
        path.pop_back();
		++iter;       
    }
}

/*find N closed points among demand points*/
void BFS(Graph *G,int i,int visit[],int closest[],TreeNode *tree,int ClosedNodeNum)
{
 	 arcnode *node ;
 	 TreeNode *tnode;
 	 queue<int> q;
 	 map<int,int> Map;
 	 int count = 0;
 	 visit[i] = 1;     
 	 q.push(G->v[i].vex); 
 	 while( !q.empty())
	 {
	     int k = q.front();
		 q.pop();
         node  = G->v[k].next;
         tnode = find_node(tree,k);
	 	 while(NULL != node)
	 	 {
 			    if(visit[node->vertex] == 0)       //if haven't visit the point
 			    {
				 	visit[node->vertex] = 1;
				 	TreeNode *child = new TreeNode();
					child->vertex = node->vertex;
					child->weight = node->weight;
					tnode->child.push_back(child); //added to the parent's children 
				 	if(false == is_demand_vertex(node->vertex)) 
				 	{
	 				 	q.push(node->vertex);					 	
					}
					else
					{   
					 	closest[++count] = node->vertex;
			 		}				  
  				}
  				node = node->next;	
		 }		 	
		 if(count >= ClosedNodeNum) break;  
  	 } 
	 closest[0] = count; //used closest[0]  to store the vertex's nums of the path
}

// global means to find the route from src & dst or just from two points
void shortest_path_from_A2B(int src,int dst,map <int,int> &mapInfo,vector <int> &vec,Graph *G,int &weight,bool global)
{
	 arcnode  	*node ;
 	 TreeNode 	*tnode;
 	 static  int  _weight = 0;
     static  vector <int> _vec;
     
     node = G->v[src].next;
     mapInfo[src] = 1;
     _vec.push_back(src);
     while(NULL != node)
     {
     	 int start = node->vertex;
     	 int w     = node->weight;
     	 _weight += w;  // add weight first
     	 
     	 if(mapInfo[start] != 1 && false == is_demand_vertex(start))
     	 {
     	 	shortest_path_from_A2B(start,dst,mapInfo,vec,G,weight,global);

		 }  //pass_all_demand_vertex(mapInfo,dst);
    
     	 else if(mapInfo[start] != 1 && true == is_demand_vertex(start))
     	 {
     	 		// save path & weight
	     	 	if (start == dst)
	     	 	{
		     	 	if (global == true && true == pass_all_demand_vertex(mapInfo,start))
		     	 	{    
					  	if (weight > _weight )
		     	 		{
		     	 			weight = _weight;
		     	 			vec = _vec;		
						}
								
					} else if (global == false)
					{
					  	if (weight > _weight)
		     	 		{
		     	 			weight = _weight;
		     	 			vec = _vec;		
						}
					}		
				} else if(global == true) 
				
				{
		     	 	shortest_path_from_A2B(start,dst,mapInfo,vec,G,weight,global);
				}
		 }
		
		_weight -= w;	 // sub weight finally	
		node = node->next;
	}
	 mapInfo[src] = 0;
	 _vec.pop_back();   	 
}

// to do:    	  
void init_vex_path(DemandVexInfo &vexpath,vector <NodeInfo> path)
{
 	 int weight = 0;
 	 for(vector <NodeInfo>::iterator iter = path.begin(); iter < path.end();iter++)
 	 {
	  			weight += (*iter).weight;
	 }
	 vexpath.weight = weight;
	 vexpath.srcvex = (*(path.begin())).vertex;
	 vexpath.dstvex = (*(path.end()-1)).vertex;
	 vexpath.node   = path;
} 	  
   
void get_path_info(TreeNode *tnode,vector <NodeInfo> p,vector<DemandVexInfo> &PathInfo,int count)
{
    vector <NodeInfo> path = p; 
    int    parent = tnode->vertex;
    
    vector<TreeNode*>::iterator begin = tnode->child.begin();
    vector<TreeNode*>::iterator iter  = tnode->child.begin();
    while(iter < tnode->child.end())
    {
	    NodeInfo info;
	    info.vertex = (*iter)->vertex;
	    info.weight = (*iter)->weight;
        path.push_back(info);
        if( false == is_demand_vertex(info.vertex))
        {
         	get_path_info(*iter,path,PathInfo,count);
        }
        else
        {
		 	DemandVexInfo vexpath;		 	   
			// the parent was not  not a origin point ,so it was wrong to set connected_map here  
			// connected_map[parent][(*iter)->vertex] = 1; //marked connected between other demand points
			// cout << "("<<parent<<","<<(*iter)->vertex<<")"<<" ";    
			// if(path[0].vertex != path.back().vertex)  
			vexpath.node = path;   		
			if(count <= 15)
			{
				PathInfo.push_back(vexpath);
			}			
			else if(count > 15 && count % 3 == 1)
			{		
	 			PathInfo.push_back(vexpath);
			} 	
		 	count++;			
        } 
        path.pop_back();
		++iter;       
    }
}
                                                     
void map_init(vector <vector<DemandVexInfo> > pathInfo,map <int,vector<DemandVexInfo> >  &map2Vector ,DemandVertex demand) 
{	 
 	 for(int i = 0;i < DemandVexSize;i++)
 	 {
	  	 map2Vector[demand.demand[i]] = pathInfo[i];
     }    
}

bool pass_all_demand_vertex(map <int,int> &mapInfo ,int vex)
{
 	 bool flag = true;
   	 // to do ?
   	 mapInfo[vex] = 1;
 	 //if (mapInfo[demandVex.dstvex] == 1) flag = false;
 	 for (vector<int>::iterator iter = demandVex.demand.begin(); iter < demandVex.demand.end(); iter++)
 	 {
	  	 if(mapInfo[(*iter)] != 1) flag = false;
     }
	 mapInfo[vex] = 0;
     return flag;	  	 
} 

bool vertexs_in_map(map<int,int> &mapInfo,DemandVexInfo pathInfo,list <int> &pathList)
{    
 
     for(vector<NodeInfo>::iterator iter = pathInfo.node.begin(); iter < pathInfo.node.end(); iter++)
     {
	 	 if(mapInfo[(*iter).vertex] == 1) return true;
	 }

  	 for(vector<NodeInfo>::iterator iter = pathInfo.node.begin(); iter < pathInfo.node.end()-1; iter++)
     {
	 	mapInfo[(*iter).vertex] = 1;
	 	pathList.push_back((*iter).vertex);
	 }	 
	 return false;
}

void vertexs_out_map(map<int,int> &mapInfo,DemandVexInfo pathInfo, list <int> &pathList)
{
     for(vector<NodeInfo>::iterator iter = pathInfo.node.begin(); iter < pathInfo.node.end()-1; iter++)
	 {
	 	 mapInfo[(*iter).vertex] = 0;
	 	 pathList.pop_back();
	 }
}

bool randmom_map_all_set(int Map[],int size)
{
 	 for(int i = 0;i < size;i++)
 	 {
 	 		 if(Map[i] == RESET)
 	 		 		   return false;
     } 
    return true;
}

void clear_randmom_map(int Map[],int size)
{
 	 for(int i = 0;i < size;i++)
 	 {
		 Map[i] = RESET;
     } 
}

void replace_shortest_route(vector <PathsVector> &GlobalVec,Graph *G)
{
	vector <PathsVector> vect = GlobalVec;
	vector <NodeInfo> Info;
	map <int,int> mapInfo;
	int total_weight_prev = 0;
	int total_weight_next = 0;
	
	for(int i = 0;i < vect.size() - 1;i++)
	{	
		mapInfo[vect[i].src] = 1;
		mapInfo[vect[i].dst] = 1;
    	for(int j = 0; j < vect[i].path.size();j++)
    	{
    		mapInfo[vect[i].path[j]] = 1;
		}	
	}
	
	for(int i = 0;i < vect.size() - 1;i++)
	{
		mapInfo[vect[i].src] = 0;
		mapInfo[vect[i].dst] = 0;
    	for(int j = 0; j < vect[i].path.size();j++)
    	{
    		mapInfo[vect[i].path[j]] = 0;
		}
		
		Info.clear();
		dijkstra(G,vect[i].src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
		record_route(vect[i].src,vect[i].dst,Path_dijkstra,Info,G);
		
		if(Path_dijkstra[vect[i].dst] != NIL)
		{
				//if(Dis_dijkstra[vect[i].dst] < vect[i].weight) 
				{
					vect[i].path.clear();
					total_weight_prev  += vect[i].weight;
					vect[i].weight = Dis_dijkstra[vect[i].dst];
					total_weight_next  += vect[i].weight;
					for(int n = 1; n < Info.size(); n++)
					{
						vect[i].path.push_back(Info[n].vertex);
						mapInfo[Info[n].vertex] = 1;
					}
					mapInfo[vect[i].src] = 1;
					mapInfo[vect[i].dst] = 1;	
					continue;
				}		
		}
		else 
		{
			cout << "error" <<endl;
			return;
		}
			
		mapInfo[vect[i].src] = 1;
		mapInfo[vect[i].dst] = 1;
		for(int j = 0; j < vect[i].path.size();j++)
		{
			mapInfo[vect[i].path[j]] = 1;
		}
	}
	
	if(total_weight_next < total_weight_prev) 
	{
		GlobalVec = vect; 
		cout << "replace1 " <<endl;
	} else
	{
		vect = GlobalVec;
	}
	
	mapInfo.clear();
	total_weight_prev = total_weight_next = 0;
	for(int i = 0;i < vect.size() - 1;i++)
	{	
		mapInfo[vect[i].src] = 1;
		mapInfo[vect[i].dst] = 1;
    	for(int j = 0; j < vect[i].path.size();j++)
    	{
    		mapInfo[vect[i].path[j]] = 1;
		}	
	}
	
	for(int i = vect.size() - 2;i >= 0 ;i--)
	{
		mapInfo[vect[i].src] = 0;
		mapInfo[vect[i].dst] = 0;
    	for(int j = 0; j < vect[i].path.size();j++)
    	{
    		mapInfo[vect[i].path[j]] = 0;
		}
		
		Info.clear();
		dijkstra(G,vect[i].src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
		record_route(vect[i].src,vect[i].dst,Path_dijkstra,Info,G);
		
		if(Path_dijkstra[vect[i].dst] != NIL)
		{
				//if(Dis_dijkstra[vect[i].dst] < vect[i].weight) 
				{
					vect[i].path.clear();
					total_weight_prev  += vect[i].weight;
					vect[i].weight = Dis_dijkstra[vect[i].dst];
					total_weight_next  += vect[i].weight;
					for(int n = 1; n < Info.size(); n++)
					{
						vect[i].path.push_back(Info[n].vertex);
						mapInfo[Info[n].vertex] = 1;
					}
					mapInfo[vect[i].src] = 1;
					mapInfo[vect[i].dst] = 1;	
					continue;
				}		
		}
		else 
		{
			cout << "error" <<endl;
			return;
		}
			
		mapInfo[vect[i].src] = 1;
		mapInfo[vect[i].dst] = 1;
		for(int j = 0; j < vect[i].path.size();j++)
		{
			mapInfo[vect[i].path[j]] = 1;
		}
	}
	
	if(total_weight_next < total_weight_prev) 
	{
		GlobalVec = vect; 
		cout << "replace2 " <<endl;
	}
}

static void vector_to_list(vector <PathsVector> vect,list<int> &pathList)
{
	pathList.clear();
	for(int i = 0;i < vect.size();i++)
	{
		pathList.push_back(vect[i].src);
		for(int j = 0; j < vect[i].path.size();j++)
			pathList.push_back(vect[i].path[j]);	 
	}
}

static void format_path_info(list<int> &pathList,Graph *G,vector <PathsVector> &vect)
{
	map  <int,int>   mapInfo;
	list<int>::iterator end;
	int  weight;
	
	for(list<int>::iterator it = pathList.begin(); it != pathList.end();it++) 	
	{
		mapInfo[*it] = 1;
	} 

	for(list<int>::iterator it = pathList.begin(); it != pathList.end();it++) 
	{	
		if(true == is_demand_vertex(*it))
		{
			if(*it == demandVex.srcvex) 
			{
				PathsVector p;
				p.src = *it;
				p.dst = *it;
				p.weight = 0;
				vect.push_back(p);
				end  = it;
			} else 
			{
				PathsVector p1,p2;
				p1 = vect.back();
				vect.pop_back();
				for(list<int>::iterator l = end; l != it;l++)  // end :src it: dst 
				{
					list<int>::iterator next = l;
					if (l != end) 
					{
						p1.path.push_back(*l);
					} 
					p1.weight += get_edge_weight(G,*l,*(++next));
				}
				p1.dst = *it;    // dst of p1
				vect.push_back(p1);
				p2.src 		= *it;
				p2.src          = *it;
				p2.weight 	= 0;
				vect.push_back(p2);
				end = it;
			}
		}
	}
	//replace_shortest_route(vect,G,mapInfo);
}

unsigned int  dfs_loop;
bool 	      dfs_timeout_flag;

int main_loop(int startVertex,list<int> &FinalList,Graph *G)
{
	int local_best = _INFINITY ;
	int local_weight;
	time_limit = clock();
	list<int> localList;
	bool changeFlag = false;
	int strategyFlag = STRATEGY_ONE;
	bool sys_in_stagnation = false;
	int  sys_in_stagnation_cnt = 0;
	
	while(1)
	{
		weight_t = local_best;
		dfs_timeout_flag = false;
		BestOfAll  = false;
		map <int,int > MapTemp;
		list <int>myList;
		int  miniweight = 0;
		int  VertexNum  = 0;
		
		evaporate_pheromone();
		
		update_roulette();
		
		if(sys_in_stagnation_cnt++ > 5)
		{
			sys_in_stagnation = true;
		    sys_in_stagnation_cnt = 0;	
		}
		
		if(sys_in_stagnation == true)  
		{
			//sys_in_stagnation = false;
			ant_sys_stagnation_handler();
			//cout << "sys_in_stagnation" <<endl;
		}
		
		local_weight    = DFS(MapTemp,startVertex ,VertexNum,myList,localList,miniweight,G,strategyFlag,sys_in_stagnation);
		if (_INFINITY != local_weight)
		{	
			if(local_best > local_weight)
			{
				local_best 	= local_weight;
				FinalList 	= localList;
				sys_in_stagnation_cnt = 0;
			}
		 	//return local_weight;
		}

#ifdef USING_STRATEGY_BOTH
		if(false == changeFlag && (clock() - time_limit) / CLOCKS_PER_SEC >= OUT_STRATEGY_ONE && local_best == _INFINITY )
		{
			changeFlag = true;
			strategyFlag = STRATEGY_TWO;
			//map_init(_pathInfo,map2Vector,demandVex);
			map_init(pathInfo,map2Vector,demandVex);
		}  else
		{
			map_init(pathInfo,map2Vector,demandVex);
		}
#endif
		if((clock() - time_limit) / CLOCKS_PER_SEC >= OUT_MINISEC && local_best != _INFINITY) 
		{
			cout << "dfs_loop:" << dfs_loop << endl;
			return local_best;	
		}			
	}
}

void dfs_loop_timeout(void)
{
	if(dfs_loop++ % DFS_LOOP_TIMEOUT == 0)
	{
		dfs_timeout_flag = true;
	}		
}

// if the vertex connected to the last demandVex.dst all in map,then cut this path off 
static inline bool key_vertexs_wrong_in_map(vector <int> &VIPVertex,map<int,int> &Map,int depth)
{
	int count = 0;

	if(depth < demandVex.demand.size() / 2)
	{
		for(vector <int>::iterator it = VIPVertex.begin();it < VIPVertex.end();it++)
		{
			if (Map[*it] == 1)
			{
				count++;
			} 
		}
		
		if(count >= VIPVertex.size()) return true;
		count = 0;
		for(vector <int>::iterator it = VIPVertex.begin();it < VIPVertex.end();it++)
		{
			if (Map[*it] == 0)
			{
				count++;
			} 
		}	
		if(count <= 0) return true;	
	} 

	return false;
}


void put_vertex_in_tabu(int vertex,int depth,vector<TabuStruct> &TabuTab)
{
	TabuStruct tabu;
	tabu.depth = depth;
	tabu.vertex = vertex;
	
	if(DemandVexSize - depth > 20) TABUSIZE = 7;    // 8  7 
	else if(DemandVexSize -depth > 15) TABUSIZE = 7;// 5  5
	else if(DemandVexSize -depth > 8) TABUSIZE = 3 ; // 3 3
	else TABUSIZE = 0;
	if(TABUSIZE) TabuTab.push_back(tabu);
	
	//if(TabuTab.size() > TABUSIZE)
	//	TabuTab.erase(TabuTab.begin());
	while(TabuTab.size() > TABUSIZE)
	{
		TabuTab.erase(TabuTab.begin());
	}
}

bool is_vertex_in_tabu(int vertex,int depth,vector<TabuStruct> TabuTab)
{
	bool flag = false;
	for(vector<TabuStruct>::iterator it=TabuTab.begin();it < TabuTab.end();it++)
	{
		if((*it).vertex == vertex) 
		{
			flag = true;
		}			
	}
	//if(depth <= (*it).depth + TABUSTEP && depth >= (*it).depth-TABUSTEP) flag = true;
	return flag;	     	
}

static inline double random_value(void)
{	
	return  rand()/(double)(RAND_MAX);
}

//roulette_prob
static inline int find_roulette_index(int vertex,int Map[])
{   
    double f;
	int index = 0;
	vector <double> vec = roulette_prob[vertex]; 
	f = random_value();
	for(vector <double>::iterator iter = vec.begin();iter != vec.end();iter++,index++)
	{
		if( (*iter) > f)
		{
		#if 0
			if(Map[index])
			{
				int j = 0;
				while(Map[j] && j < vec.size()-1) { j++; }
				index = j;
			}
		#endif
			return index;
		}
	}
	return index-1; // should be index-1 this time
}

//#define ALPHA  1.0
//#define BETA   2.0
static void update_roulette(void)
{
	roulette_prob.clear();
	
	for(int i = 0;i < DemandVexSize;i++)
	{ 
		double Ptotal = 0;
		vector <double> prob;
		double f = 0;
		
		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
		{			               	
			Ptotal += pow((*iter).pheromone,ALPHA) / pow((*iter).weight,BETA);	     
		} 
		//  (*iter).node.size() * PATH_LENGTH_WEIGHT
		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
		{
		// (*iter).node.size() * PATH_LENGTH_WEIGHT
			f	   += pow((*iter).pheromone,ALPHA) / pow((*iter).weight,BETA)/ (double) Ptotal;
			//cout << "@:" << f ;
			prob.push_back(f); 
		}  
		//cout << endl;
		roulette_prob.push_back(prob);
	}
	//cout << "size :" <<roulette_prob.size() << endl;
}

static void evaporate_pheromone(void)
{
	for(int i = 0;i < DemandVexSize;i++)
	{
		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++)
		{
			(*iter).pheromone = (*iter).pheromone * ROU; 		 //evaporate pheromone
			if(ant_max > 0) if((*iter).pheromone > ant_max)  (*iter).pheromone = ant_max;
			if(ant_min > 0) if((*iter).pheromone < ant_min)  (*iter).pheromone = ant_min;
		}
	}
}

void init_ant_data(void)
{
	//计算最大和最小信息素之间的比值
	cityNum=(double)DemandVexSize;
	
	dbTemp = exp(log(Pbest)/cityNum); 
	//对Pbest开N_CITY_COUNT次方
	m_dbRate=(2.0/dbTemp-2.0)/(cityNum-2.0);
	ant_max = 0;
	ant_min = 0;
}

// ant system in stagnation
void ant_sys_stagnation_handler(void)
{
	for(int i = 0;i < DemandVexSize;i++)
	{
		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++)
		{
			if(ant_max > 0 && ant_min > 0)
			{
				 (*iter).pheromone = (*iter).pheromone + LUO * (ant_max - (*iter).pheromone); 	 //evaporate pheromone
				 if((*iter).pheromone > ant_max)  (*iter).pheromone = ant_max;
				 if((*iter).pheromone < ant_min)  (*iter).pheromone = ant_min;
			}
		}
	}	
}

int get_list_weight(list<int> FinalList,Graph *G)
{
	list<int>::iterator _iter;
	double weight = 0;
	
	 list <int>::iterator iter_t;
	 for(list <int>::iterator iter = FinalList.begin();iter != FinalList.end();iter++)
	 {	  
		if(iter != FinalList.begin())
		{	   		       
			weight += get_edge_weight(G,*(iter_t),*(iter));
		}
		iter_t = iter;
	}
	return weight;	
}

void update_route_pheromone(list<int> FinalList,bool best,Graph *G)
{

	double weight = get_list_weight(FinalList,G);
	
	ant_max = 1.0 / (weight * (1.0-ROU)) ; //update only when find best route
	ant_min = ant_max * m_dbRate;
	
	list <int>::iterator _iter;
	for(list<int>::iterator it = FinalList.begin(); it != FinalList.end();it++) 
	{
		int id = demandVertexIndexMap[*it];
		_iter = it;
		_iter++;
		if (_iter == FinalList.end()) break;
		
		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[id].begin(); iter < pathInfo[id].end();iter++)
		{
			if(*_iter == (*iter).node.back().vertex)  
			{	
				//if(best == true)
				(*iter).pheromone  = (*iter).pheromone + 1.0 / weight; 
				//else 
				//(*iter).pheromone  = (*iter).pheromone + 1.0 / weight * 0.1; 				
				if((*iter).pheromone > ant_max)  (*iter).pheromone = ant_max;
				if((*iter).pheromone < ant_min)  (*iter).pheromone = ant_min;
			}
		}
	}
		
	//if (best == true)
	//{
	//	ant_max = 1.0 / (weight * (1.0-ROU)) ; //update only when find best route
	//	ant_min = ant_max * m_dbRate;
	//}
}

//LastConnectVertex
//note:  the first points was in discribe as srcdst int the struct
int DFS(map <int,int> &Map,int startVertex ,int &VertexNum ,list<int> &pathList,list<int> &FinalList, int miniweight,Graph *G,int flag,bool &stagnation_flag)
{
	static int  local_best_cnt;
	static bool local_best_flag = false;
	int count = 0;

	dfs_loop_timeout();
	
	if(BestOfAll == true || local_best_flag == true)
	{
		BestOfAll = false;
		local_best_flag = false;
		cout <<"----------------"<<endl;
		update_route_pheromone(FinalList,true,G);
	} 
	else if( (VertexNum >= DemandVexSize -2 && FinalList.size() == 0) || ( stagnation_flag == true && VertexNum >= DemandVexSize -3 ) )  
	{
		if(true == stagnation_flag) cout << "in stagnation" << endl;
		local_best_flag = false;
		stagnation_flag = false;
		update_route_pheromone(pathList,false,G);		
	}
	
	if(true == dfs_timeout_flag) return weight_t;

#ifdef INIT_SEED_CLOCKS 	 
	 srand((unsigned int)clock());
#else
	 srand(dfs_loop);
#endif
	 
     vector <DemandVexInfo> vec = map2Vector[startVertex]; 
     int size = vec.size(); 
     if(size == 0) 
	 {	
	  		return _INFINITY;
	 }
	 
     VertexNum++;		   //marked have pass througth ademand point
     Map[startVertex] = 1; //mark the start point  
     pathList.push_back(startVertex);
	 do
	 {	
          int offset;     
#ifdef  USING_STRATEGY_BOTH

		if(VertexNum < DemandVexSize - 6)
		{
			if(count++ > 30) break;
			offset = find_roulette_index(demandVertexIndexMap[startVertex],randmomMap[startVertex]);
		}
		else
		{
			offset =  rand()% size;  	
		}
		
      //if(count++ > 20) break;	
   	  //if(flag == STRATEGY_ONE) offset = find_roulette_index(demandVertexIndexMap[startVertex]);					 
	  // else offset = rand()% size; 		//random index
	  //offset = rand() % size;
	  
#else 

      offset =  rand()% size;  
#endif
	  if(randmomMap[startVertex][offset] == RESET)
	  {
			 randmomMap[startVertex][offset] = SET;
			 DemandVexInfo Info = vec[offset];
	
			 if(true == vertexs_in_map(Map,Info,pathList)) continue;
			  
	
			 if(true == pass_all_demand_vertex(Map,Info.dstvex))
			 {
				if (local_best_cnt++ % 5 == 0) local_best_flag = true;
			 }
			 
			 if((miniweight + Info.weight) >= weight_t) 
			 {
			 		vertexs_out_map(Map,Info,pathList);   
			 		continue;
			 }

			 miniweight += Info.weight; 

			 if(true == pass_all_demand_vertex(Map,Info.dstvex))
			 {
			      if(Info.dstvex == demandVex.dstvex) 
				  {
				   		if(miniweight < weight_t) 
			   			{
			   				  vector <PathsVector>  GlobalVec;
				 			  weight_t = miniweight;
				 			  int m_cnt = 0;
				 			  //find  
				              FinalList = pathList;
				              BestOfAll = true;
				              local_best_flag = false;
				              ////////////////////////////////////////
				              FinalList.push_back(demandVex.dstvex);	 
				              format_path_info(FinalList,G,GlobalVec);
				              
	  						  while(m_cnt++ < 50) mutate(GlobalVec,G,true);
	  						  
	                          vector_to_list(GlobalVec,FinalList);
	                          int _weight = get_list_weight(FinalList,G);
	                          
	                          weight_t = _weight;
	                          FinalList.pop_back();
	                          ////////////////////////
				        } 	
		          }			 
			 }
			 
			 DFS(Map,Info.dstvex,VertexNum,pathList,FinalList,miniweight,G,flag,stagnation_flag); 

	 		 vertexs_out_map(Map,Info,pathList);	 		
	 		 miniweight -= Info.weight;	 
	  }	
	   	 								 
	 } while(randmom_map_all_set(randmomMap[startVertex],size) == false && false == dfs_timeout_flag);

	 //put_vertex_in_tabu(startVertex,VertexNum,TabuTab);	 
	 clear_randmom_map(randmomMap[startVertex],size);
	 Map[startVertex] = 0; 
	 pathList.pop_back();
	 VertexNum--;	
	 
	 return weight_t;
}

void set_connected_map(int src,int dst)
{
	connected_map[src][dst] = 1;
}

bool connected_in_map(int src,int dst)
{
	return (connected_map[src][dst] == 1) ? true: false;
}

void int2str(const int &int_temp,string &string_temp)  
{  
        stringstream stream;  
        stream<<int_temp;  
        string_temp=stream.str();   // stream>>string_temp  
}  

void set_dynamic_notes_num(Graph *G)
{
 	 if(G->numofVex <= 40)
 	 {
	  DynamicChangeNode = 12;	
	 } 
	 else if(G->numofVex > 40 && G->numofVex <=50)
	 {
	 DynamicChangeNode = 12; 
  	 }
  	 else if(G->numofVex > 50 && G->numofVex <=60)
	 {
	 DynamicChangeNode = 12; 
  	 }
  	 else if(G->numofVex > 60 && G->numofVex <=80)
	 {
	 DynamicChangeNode = 12; 
  	 }else if(G->numofVex > 80 && G->numofVex <=250)
	 {
	 DynamicChangeNode = 20; 
	 
  	 } else
  	 {
	    DynamicChangeNode = 50; 	   
     }
}

static bool contain_in_paths(vector<vector<DemandVexInfo> > pathInfo,int vertex)
{
 	 for(int i = 0;i < DemandVexSize;i++)
     { 	  
	  	  for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
	  	  {
				 for(vector<NodeInfo>::iterator it = (*iter).node.begin(); it < (*iter).node.end(); it++)
				 {
     					if(vertex == (*it).vertex) return true;
                 }               	        	 	         
	      }  
   	 } 	
   	 return false;
}


void init_visit_array(void)
{
   for(int i = 0;i < DemandVexSize;i++)
   {
 	   for(int j = 0; j < MAX_VERTEX_NUM;j++)
	   {
 	   		 visit[i][j] = 0;
			 _visit[i][j] = 0;
	   }
   }
}

void add_demand_in_map(map <int,int> &Map,int vertex)
{
	Map[vertex] = 1;
}

void del_demand_in_map(map <int,int> &Map,int vertex)
{
	Map[vertex] = 0;
}


void find_closest_points(Graph *G,string &outStr)
{
 	 arcnode *node;
 	 int i,j; 
 	 map <int,int > MapTemp;
	 int depth = 0,miniweight = 0;
	 list <int>myList;
	 vector <PathsVector>  GlobalVec;
	 init_visit_array();
     for(i = 0;i < DemandVexSize;i++)
     	   for(j = 0;j < 20;j++)
     	   		 closest[i][j] = 0;  
     	   		
	 set_dynamic_notes_num(G);
	 // add dstvex in demandmap
	add_demand_in_map(MapTemp,demandVex.dstvex);
     for(i = 0;i < DemandVexSize;i++)
     {
     	 add_demand_in_map(MapTemp,demandVex.demand[i]);
 	     init_tree_node(treeNode[i],demandVex.demand[i]);
	     init_tree_node(_treeNode[i],demandVex.demand[i]);
#ifdef USING_STRATEGY_FIRST
   		 BFS(G,demandVex.demand[i],visit[i],closest[i],&treeNode[i],DynamicChangeNode);
#elif  USING_STRATEGY_SECOND
		_BFS(G,demandVex.demand[i],visit[i],&treeNode[i]);
#else
		BFS(G,demandVex.demand[i],visit[i],closest[i],&treeNode[i],DynamicChangeNode);
 		_BFS(G,demandVex.demand[i],_visit[i],&_treeNode[i]);

#endif
	 
  	 }  
	 
  	 for(i = 0;i < DemandVexSize;i++)
  	 {
  	 	   int demandCnt = 0;
	  	   vector<NodeInfo> path,_path;
	  	   vector<DemandVexInfo> paths,_paths;
	  	   int count = 0;
#ifdef USING_STRATEGY_FIRST
	  	   get_path_info(&treeNode[i],path,paths);
#elif  USING_STRATEGY_SECOND
	  	   _get_path_info(&treeNode[i],path,paths,demandCnt);
#else 
		  get_path_info(&treeNode[i],path,paths,count);
		  _get_path_info(&_treeNode[i],_path,_paths,demandCnt);
		  _pathInfo.push_back(_paths);
#endif
	  	   pathInfo.push_back(paths);

     	}
      
	for(i = 0;i < DemandVexSize;i++)
	{ 
		demandVertexIndexMap[demandVex.demand[i]] = i; // roulette_prob index map	  
		
		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
		{     // DemandVexInfo.node[0].vertex not the begin vertex  but the last node of prev path 
			int weight = 0,dstvex = 0;  
			InDegreeCnt[(*iter).node.back().vertex] = InDegreeCnt[(*iter).node.back().vertex] + 1;

			for(vector<NodeInfo>::iterator it = (*iter).node.begin(); it < (*iter).node.end(); it++)
			{

				dstvex  = (*it).vertex; 
				weight += (*it).weight;	       
			} 

			(*iter).srcvex = demandVex.demand[i];

			(*iter).dstvex = dstvex;
			(*iter).weight = weight; 	
			set_connected_map((*iter).srcvex,(*iter).dstvex);
			(*iter).pheromone =  1.0 / ((*iter).weight * 50); // init pheromone 	        	 	         
		}  
	} 	 

#if 0  //need 400 ms cost too much 

	for( i = 0;i < DemandVexSize;i++)
	{
		vector <NodeInfo> Info;
		
		del_demand_in_map(MapTemp,demandVex.demand[i]);
		for(int j = 0;j < DemandVexSize;j++)
		{
			del_demand_in_map(MapTemp,demandVex.demand[j]);
			
			dijkstra(G,demandVex.demand[i],Dis_dijkstra,Path_dijkstra,Done_dijktra,MapTemp);
			
			add_demand_in_map(MapTemp,demandVex.demand[j]);
			
			record_route(demandVex.demand[i],demandVex.demand[j],Path_dijkstra,Info,G);
			
			if(Path_dijkstra[demandVex.demand[j] ]!= NIL)	set_connected_map(demandVex.demand[i],demandVex.demand[j]);		
		}
		
		del_demand_in_map(MapTemp,demandVex.dstvex);
		
		dijkstra(G,demandVex.demand[i],Dis_dijkstra,Path_dijkstra,Done_dijktra,MapTemp);
		
		add_demand_in_map(MapTemp,demandVex.dstvex);
		
		record_route(demandVex.demand[i],demandVex.dstvex,Path_dijkstra,Info,G);
		
		if(Path_dijkstra[demandVex.dstvex] != NIL)	set_connected_map(demandVex.demand[i],demandVex.dstvex);
		
		add_demand_in_map(MapTemp,demandVex.demand[i]);	
	}
#endif
	
	
#ifdef    USING_STRATEGY_BOTH
	for(i = 0;i < DemandVexSize;i++)
	{ 	  
		if(pathInfo[i].size() <= 1) OutDegreeOne[demandVex.demand[i]] = 1;
		if(InDegreeCnt[demandVex.demand[i]] <= 1) InDegreeOne[demandVex.demand[i]] = 1;

		for(std::vector<DemandVexInfo>::iterator iter = _pathInfo[i].begin(); iter < _pathInfo[i].end();iter++) 
		{     // DemandVexInfo.node[0].vertex not the begin vertex  but the last node of prev path 
			int weight = 0,dstvex = 0;  
			for(vector<NodeInfo>::iterator it = (*iter).node.begin(); it < (*iter).node.end(); it++)
			{

				dstvex  = (*it).vertex; 
				weight += (*it).weight;	       
			} 

			(*iter).srcvex = demandVex.demand[i];

			(*iter).dstvex = dstvex;
			(*iter).weight = weight; 	
			//set_connected_map((*iter).srcvex,(*iter).dstvex);	        	 	         
		}  
	} 
#endif			

#if  0	 

	for(i = 0;i < DemandVexSize;i++)
	{ 
		double total_weight = 0;
		vector <double> prob;
		double f = 0;

		demandVertexIndexMap[demandVex.demand[i]] = i; // roulette_prob index map

		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
		{	// demand points all the weight of every route				               	
			total_weight += (double)1 / ((*iter).weight + (*iter).node.size() * PATH_LENGTH_WEIGHT);	 
			//cout << (double )1 / (*iter).weight <<"--";	         
		} 

		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
		{
			//f +=  ((double)1/(*iter).weight) / (double) total_weight;
			f +=  (double)1 / ((*iter).weight + (*iter).node.size() * PATH_LENGTH_WEIGHT) / (double) total_weight;
			prob.push_back(f); 
		}  
		roulette_prob.push_back(prob);
	}
	
#endif

    init_ant_data(); // init ant imformation
    
	for(i = 0;i < DemandVexSize;i++)
	{
		for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
		{	vector<NodeInfo>  node_vec = (*iter).node;
			if(node_vec.back().vertex == demandVex.dstvex) 
			{
				LastConnectVertex.push_back((*iter).srcvex);
			}
		}
	}
	 
     cout << "clockpersec" <<CLOCKS_PER_SEC << endl;
   	 cout << "vertex:"     <<G->numofVex    << endl;
	 cout << "result:"     <<endl;   
	 map_init(pathInfo,map2Vector,demandVex);   
//////////////////////////////////////////////////////////////
   	
     list<int> FinalList; 	
	 int weight = _INFINITY;  
	 
	 if(G->numofVex <= 45) 
	 {
		 map <int,int> myInfo;
		 vector <int> myVec;
		 shortest_path_from_A2B(demandVex.srcvex,demandVex.dstvex,myInfo,myVec,G,weight,true);
		 for (vector<int>::iterator it = myVec.begin(); it < myVec.end(); it++)
		 {
		 	FinalList.push_back(*it);
		 }
		 
	 } else
	 {
	 	weight = main_loop(demandVex.srcvex,FinalList,G); 
	 }
		

	 cout << "minest weight:"<< weight<<" steps: "<< FinalList.size()+1 <<" average w:" << average_weight<<"demand.size:" << DemandVexSize <<endl; 
	 	 
	 if(weight != _INFINITY)
	 {
	 	 weight = 0;
		 FinalList.push_back(demandVex.dstvex);	 
		 
#ifdef ENABLE_DIJKSTRA
		 if(G->numofVex > 100)
		 {
		 	format_path_info(FinalList,G,GlobalVec);
		 	while(mutate_cnt ++ < MAX_GEN ) 
			{
				
				mutate(GlobalVec,G,true);
				if(mutate_cnt % 30 == 0)  replace_shortest_route(GlobalVec,G);
				
			}
		 	//while(mutate_cnt ++ < MAX_GEN && (clock() - time_limit) < MAX_OUTTIME_LIMIT * CLOCKS_PER_SEC) mutate(GlobalVec,G,true);
			//vector <vector <PathsVector> > pop;
			//ga(GlobalVec,pop,G);
		 	vector_to_list(GlobalVec,FinalList);	
		 } 	 
#endif
		 list <int>::iterator iter_t;
		 for(list <int>::iterator iter = FinalList.begin();iter != FinalList.end();iter++)
		 {
			string str;	  
			//cout << *iter <<"#";		  
			if(iter != FinalList.begin())
			{	   		       
				int num = get_edge_number(G,*(iter_t),*(iter));
				weight += get_edge_weight(G,*(iter_t),*(iter));
				int2str(num,str);
				outStr+=str+'|';
			}
			iter_t = iter;
	  	 }
	  	 outStr = outStr.substr(0,outStr.size()-1);
	  	 refresh_write_buff(outStr);
	 }	 
	 cout << "final weight:" << weight <<endl;
}

extern char g_result[];
void refresh_write_buff(string outStr)
{
 	 const char *ch = outStr.c_str();
 	 
 	 for(int i =0;i < outStr.size();i++)
 	 		 g_result[i] = ch[i];
}

int get_edge_weight(Graph *G,int vstart,int vend)
{
	arcnode *node = G->v[vstart].next;
	while(node != NULL)
	{
		if(node->vertex == vend)
		{
		 return node->weight;
		}
		node = node->next;
	}	
	return _INFINITY;
}

int get_edge_number(Graph *G,int vstart,int vend)
{
 	arcnode *node = G->v[vstart].next;
	while(node != NULL)
	{
		if(node->vertex == vend)
		{
		 return node->arc;
		}
		node = node->next;
	}	
	return _INFINITY; 	
}

//dijkstra(G,demandVex.srcvex ,Dis_dijkstra,Path_dijkstra,Done_dijktra) 
typedef pair<int,int> pair_t;

void dijkstra(Graph *G,int src,int d[],int p[],int done[], map<int,int> mapInfo)
{
	priority_queue<pair_t,vector<pair_t>,greater <pair_t> > q;
	q.push(make_pair(d[src],src));
	for(int i = 0;i < 1000;i++) d[i] = (i == src ? 0:_INFINITY);
	for(int i = 0;i < 1000;i++) p[i] = NIL;
	for(int i = 0;i < 1000;i++) done[i] = 0; 
	while(!q.empty())
	{
		pair_t u = q.top();
		q.pop();
		int ver1 = u.second;     // ver current node number
		if(done[ver1] || mapInfo[ver1]) continue; // already caculate 
		done[ver1] = 1;
		arcnode *next = G->v[ver1].next;
		while (next != NULL)
		{
			int ver2 = next->vertex;
			if(!mapInfo[ver2])
			{
				int w    = get_edge_weight(G,ver1,ver2);
				if(d[ver2] > d[ver1] + w)
				{
					d[ver2] = d[ver1] + w;
					p[ver2] = ver1;  // save path info
					q.push(make_pair(d[ver2],ver2));
				}
			}				
			next = next->next;
		}			
	} 
}

// dst 
void record_route(int src,int dst,int path[],vector <NodeInfo> &Info,Graph *G)
{
 	 stack <NodeInfo> s;
 	 int node = dst;
 	 NodeInfo temp;
	 //cout << dst << "dst" <<"path[node]"<<path[node]<<endl;	 
 	 if(path[node] != NIL)
 	 {
	 	 while(path[node] != NIL)
	 	 {
		   temp.weight = get_edge_weight(G,path[node],node);
		   temp.vertex = node = path[node];
		   s.push(temp);
	 	 }
	 	 
	 	 while(!s.empty())
	 	 {
		   Info.push_back(s.top());
		   //cout << (s.top()).vertex <<"-->";
		   s.pop();
	     }
     } 
}


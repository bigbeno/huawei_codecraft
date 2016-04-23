#include "route.h"
#include "lib_record.h"
#include "multiway_tree.h"
#include "lib_time.h"
#include <ctime>

#define INFINITY  10000000
#define SET    1
#define RESET  0
#define MAX_DEMAND_VERTEX 100
#define MAX_VERTEX_NUM    600 
#define EDGE_OF_DEMAND_VERTEX 6
#define DemandVexSize   (demandVex.demand.size())
#define OUT_MINISEC     9000000
#define TIME_SLICE       1 

#define DFS_LOOP_TIMEOUT  200
//using 200 has a good result
static int weight_t = INFINITY;
static unsigned int milisec;

int  DynamicChangeNode = 6; //change according Graph info  

DemandVertex demandVex;

int visit[MAX_DEMAND_VERTEX][600];
int closest[MAX_DEMAND_VERTEX][20];
TreeNode treeNode[MAX_DEMAND_VERTEX];
vector<vector<DemandVexInfo> >   pathInfo; //very inportant contain all the infomation of the route 
map <int,vector<DemandVexInfo> > map2Vector;  //should be initial at the begining 
int randmomMap[1000][MAX_DEMAND_VERTEX] = {0};
int average_weight = 0;
int connected_map[MAX_VERTEX_NUM][MAX_VERTEX_NUM]; //to restore information between two demanded vertexs

// dijkstra needed 
int AllVexterNum [1000];
int Dis_dijkstra [1000];
int Path_dijkstra[1000];
int Done_dijktra [1000]; 

void dijkstra(Graph *G,int src,int d[],int p[],int done[]);
void record_route(int src,int dst,int path[],vector <NodeInfo> &Info,Graph *G);
int  get_edge_number(Graph *G,int vstart,int vend);
void refresh_write_buff(string outStr);
vector <int> generate_possible_route(vector<int> vertex,vector<int> &vertex_new,int endVertex,vector <NodeInfo> &Path);
int DFS(map <int,int> &Map,int startVertex ,int &VertexNum ,list<int> &pathList,list<int> &FinalList, int miniweight,Graph *G);
int main_loop(int startVertex,list<int> &FinalList,Graph *G);

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
  ptr->vertex 	= nodeInfo[2];
  ptr->weight   = nodeInfo[3]; 
  ptr->next 	= NULL;
  average_weight+= nodeInfo[3]; 
  return ptr;
}

void graph_init(Graph *G,int edge_num)
{
 	 int i ,j;
 	 for(i = 0;i < MAX_VERTEX_NUM;i++)
 	 {
	  	   G->v[i].vex  = -1;
	  	   G->v[i].next = NULL;
   	 }
   	 G->numofEdge = edge_num;
   	 G->numofVex  = -1;
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
	  	    v = &G->v[nodeInfo[VERTEX_OFFSET]];	//    
			if(v->next != NULL) //if initialed 
			{	
				arcnode *arc = v->next;
				while(arc->next != NULL){arc = arc->next;}
				arc->next = assign_node(nodeInfo);	              				   
			} else {
   			    v->vex 	= nodeInfo[VERTEX_OFFSET];
			  	v->next = assign_node(nodeInfo);  
			  	AllVexterNum[cnt++] = nodeInfo[VERTEX_OFFSET];
            }	  	     	   
	 }	 
   
	 get_demand_vertex(demand,Vertex); 
	 //caculated average weight
	 average_weight = average_weight / G->numofEdge;
}

void destory_graph(Graph *G)
{
	int i;
	arcnode *node,*ptr; 
	for(i = 0;i < MAX_VERTEX_NUM;i++)
	{	   
		node = G->v[i].next;
		while (node != NULL)
		{	ptr = node;
			node = node->next;  
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
 	 node.weight = 10;
}

/*find N closed points among demand points*/

void BFS(Graph *G,int i,int visit[],int closest[],TreeNode *tree)
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
 			    if(visit[node->vertex] == 0)//if haven't visit 
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
		 if(count >= DynamicChangeNode) break;  
  	 } 
	 closest[0] = count; //to store the vertex of the path
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
   	  

void get_path_info(TreeNode *tnode,vector <NodeInfo> p,vector<DemandVexInfo> &PathInfo,Graph *G)
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
         	get_path_info(*iter,path,PathInfo,G);
        }
        else
        {
		 	DemandVexInfo vexpath;
		 	
#ifdef ENABLE_DIJKSTRA
			vector <NodeInfo>  pathTemp;
			int w1 = 0,w2 = 0;	 
		 	//init_vex_path(vexpath,path);dijkstra(Graph *G,int src,int d[],int p[],int done[])	 		
	 	    dijkstra(G,(*begin)->vertex,Dis_dijkstra,Path_dijkstra,Done_dijktra);
  			record_route((*begin)->vertex,(*iter)->vertex,Path_dijkstra,pathTemp,G);		
    		if(Path_dijkstra[(*iter)->vertex] == NIL) pathTemp = path; //possible loop 
    		
    		for(vector <NodeInfo>::iterator iter = pathTemp.begin();iter < pathTemp.end();iter++)
    		{
 		   	   w1 += (*iter).weight; 
 		    }
 		  	for(vector <NodeInfo>::iterator iter = path.begin();iter < path.end();iter++)
    		{
 		   	   w2 += (*iter).weight; 
 		    }
    		if(w1 != w2)
			{
			 	  vexpath.node = pathTemp;
		 	      PathInfo.push_back(vexpath);    		
			}
#endif     
			// the parent was not  not a origin point ,so it was wrong to set connected_map here  
			//connected_map[parent][(*iter)->vertex] = 1; //marked connected between other demand points
			//cout << "("<<parent<<","<<(*iter)->vertex<<")"<<" ";    
			//if(path[0].vertex != path.back().vertex)
			{
				vexpath.node = path;   		
		 		PathInfo.push_back(vexpath);
			} 
				
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
   	 // to do? 
   	 mapInfo[vex] = 1;//
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



unsigned int  dfs_loop;
bool dfs_timeout_flag;

int main_loop(int startVertex,list<int> &FinalList,Graph *G)
{
	int local_best;
	int time_limit = clock();
	while(1)
	{
	    //milisec  = clock();
	    weight_t = INFINITY;
	    dfs_timeout_flag = false;
	    map <int,int > MapTemp;
		list <int>myList;
		int miniweight = 0;
		int  VertexNum = 0;
		local_best = DFS(MapTemp,startVertex ,VertexNum,myList,FinalList,miniweight,G);
		if (INFINITY != local_best)
			return local_best;	
		//if(clock() - time_limit > OUT_MINISEC) 
		//{
		//		cout << "dfs_loop:"<<dfs_loop<<endl;
		//		return INFINITY;	
		//}
				
	}

}

void dfs_loop_timeout(void)
{
	if(dfs_loop++ % DFS_LOOP_TIMEOUT == 0)
	{
		dfs_timeout_flag = true;
	}		
}

//note\A3\BADemandVexInfo*iter\A3\A9the first points was in discribe as srcdst int the struct
int DFS(map <int,int> &Map,int startVertex ,int &VertexNum ,list<int> &pathList,list<int> &FinalList, int miniweight,Graph *G)
{	 
#if 0	 
      if(clock() - milisec > TIME_SLICE)  return weight_t;
#else
	 dfs_loop_timeout();
	 if(true == dfs_timeout_flag) return weight_t;
#endif
	 //srand((unsigned int)clock());
	 srand(dfs_loop);
     vector <DemandVexInfo> vec = map2Vector[startVertex]; 
     int size = vec.size(); 
     if(size == 0) 
	 {	
	  		 return INFINITY;
	 }
	 
     VertexNum++;//marked have pass througth ademand point
     Map[startVertex] = 1; //mark the start point  
     pathList.push_back(startVertex);
	 do
	 {	   								 
	  int offset = rand()% size; //random index
	  if(randmomMap[startVertex][offset] == RESET)
	  {
			 randmomMap[startVertex][offset] = SET;
			 DemandVexInfo Info = vec[offset];
			 
			 if(true == vertexs_in_map(Map,Info,pathList) ) continue;

			 //if(VertexNum >= 15 && pathList.size() > VertexNum * 2)  
			 //{
			 // 		vertexs_out_map(Map,Info,pathList); 
			 // 		goto OUT;	  
		     //} 
			  
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
				 			  weight_t = miniweight;
				              FinalList = pathList;	
				        }		
		          }			 
			 }
	         DFS(Map,Info.dstvex,VertexNum,pathList,FinalList,miniweight,G); 
	 		 vertexs_out_map(Map,Info,pathList);	 		
	 		 miniweight -= Info.weight;	 
	  }
	   	 								 
	 } while(randmom_map_all_set(randmomMap[startVertex],size) == false);
OUT:	 
	 clear_randmom_map(randmomMap[startVertex],size);
	 Map[startVertex] = 0; 
	 pathList.pop_back();
	 VertexNum--;	
	 
	 return weight_t;
	 
/*  
  	 for (std::vector<DemandVexInfo>::iterator iter = vec.begin(); iter < vec.end();iter++)
  	 {	  		 	
 	    if(true == vertexs_in_map(Map,*iter,pathList) ) continue;//already passed ,change another way
 	    
        if((miniweight + (*iter).weight) >= weight_t) 
		{
		 		vertexs_out_map(Map,*iter,pathList);   
		 		continue;
		}
		
        miniweight += (*iter).weight; 

		if(true == pass_all_demand_vertex(Map,(*iter).dstvex)) 
		{
		 		#if 0
	 		//vector <NodeInfo>  pathTemp;
	 	    //dijkstra(G,(*iter).dstvex ,Dis_dijkstra,Path_dijkstra,Queue_dijktra); 		
    		if(Path_dijkstra[demandVex.dstvex] != NIL)
    		{
			   record_route((*iter).dstvex,demandVex.dstvex,Path_dijkstra,pathTemp,G);
			   miniweight += Dis_dijkstra[demandVex.dstvex]; 
			   if(miniweight < weight_t) 
			   {
 				 weight_t = miniweight;
 				 
			   }   
 		    }else
			 {
			  //	 cout <<"error"<<endl;
  	         }	
			   #endif
			  if((*iter).dstvex == demandVex.dstvex) 
			  {
			   		if(miniweight < weight_t) 
		   			{
			 			  weight_t = miniweight;
			              FinalList = pathList;	
			        }		
              }		     
		}
 		DFS(Map,(*iter).dstvex,VertexNum,pathList,FinalList,miniweight,G); 
 		vertexs_out_map(Map,*iter,pathList);
 		
 		miniweight -= (*iter).weight;
	 }
	 Map[startVertex] = 0;
	 pathList.pop_back();
	 VertexNum--;		
	 
	 return weight_t;
	 */
}


typedef struct {
	vector <int> list;
    bool collision;   //where collision of not 
    int  conflicts;   // collision total times
}path_list_t;


vector <path_list_t>  vec_list;

void init_path_list(path_list_t &list,DemandVexInfo pathInfo)
{
	list.collision = false;
	list.conflicts  = 0;
	for (vector<NodeInfo>::iterator iter = pathInfo.node.begin(); iter < pathInfo.node.end(); iter++)
    {
	 	 list.list.push_back((*iter).vertex);
	}
}
 
bool path_pruning_happened(int error_num,int step_now,int step_total)
{

	     if(step_now > 0  && step_now <= 10 && error_num >= 2) return true;
	else if(step_now > 10 && step_now <= 20 && error_num >= 3) return true;
	else if(step_now > 20 && step_now <= 30 && error_num >= 4) return true;
	else if(step_now > 30 && step_now <= 40 && error_num >= 6) return true;
	else if(step_now > 40 && step_now <= 50 && error_num >= 8) return true;
	else return false;   
}

bool add_path_in_list(DemandVexInfo pathInfo,vector<path_list_t> &vec_list,const int step_now,const int step_total)
{    
     path_list_t list_t;   
     init_path_list(list_t,pathInfo);   
	 bool flag_set    ; 
	 bool flag_reset  ;
	 int  total_times = 0;
	 bool add_path = true;
	 for(vector<path_list_t>::iterator iter = vec_list.begin(); iter < vec_list.end(); iter++)
	 {   
	 	  flag_set = false;
	 	  for(vector<int>::iterator  it_a = (*iter).list.begin();it_a < (*iter).list.end() ;it_a++) 
	 	  {
	 	  	 // cause we have already caculated the last element in vector so we begin at second elements this time
	 	  	 for(vector <int>::iterator it_b = list_t.list.begin(); it_b < list_t.list.end();it_b++) 
	 	  	 { 
	 	  	 	if(*it_b == *it_a)  
				{
					if (flag_set == false)
					{
						flag_set = true;
						(*iter).collision = true;	
						(*iter).conflicts++;     //add conflict times
					}
				}		  
			 }			
		  }
	 }
	 
   	for (vector<path_list_t>::iterator iter = vec_list.begin(); iter < vec_list.end(); iter++)
   	{
		if((*iter).collision == true) total_times++;  // caculate total conflict times
   	}
    
    if (true == path_pruning_happened(total_times,step_now,step_total)) 
    {
    	add_path = false;
	    for(vector<path_list_t>::iterator iter = vec_list.begin(); iter < vec_list.end(); iter++)
		{
			flag_reset = false;
	 	    for(vector<int>::iterator  it_a = (*iter).list.begin();it_a < (*iter).list.end() ;it_a++) 
	 	    {
	 	  	 // cause we have already caculated the last element in vector so we begin at second elements this time
	 	  	 for(vector<int>::iterator it_b = list_t.list.begin(); it_b < list_t.list.end();it_b++) 
	 	  	 { 
	 	  	 	if(*it_b == *it_a)  
				{
					if (flag_reset == false)
					{
						flag_reset = true;
						//add conflict times
						if((*iter).conflicts > 0)
							(*iter).conflicts--; 
						if((*iter).conflicts == 0)	
							(*iter).collision = false;	
						
					}
				}		  
			 }			
		   }
		}
	}
	
	if(true == add_path)  vec_list.push_back(list_t);
	 
	return add_path; // find out whether prune happened or not
}

void sub_path_in_list(DemandVexInfo pathInfo,vector<path_list_t> &vec_list)
{
	path_list_t list_t;
	bool flag_reset  ;
    list_t = vec_list.back();
    
	vec_list.pop_back(); // delete the last element
    for(vector<path_list_t>::iterator iter = vec_list.begin(); iter < vec_list.end(); iter++)
	{
		flag_reset = false;
 	    for(vector<int>::iterator  it_a = (*iter).list.begin();it_a < (*iter).list.end() ;it_a++) 
 	    {
 	  	 // cause we have already caculated the last element in vector so we begin at second elements this time
 	  	 for(vector<int>::iterator it_b = list_t.list.begin(); it_b < list_t.list.end();it_b++) 
 	  	 { 
 	  	 	if(*it_b == *it_a)  
			{
				if (flag_reset == false)
				{
					flag_reset = true;
					//add conflict times
					if((*iter).conflicts > 0)
						(*iter).conflicts--; 
					if((*iter).conflicts == 0)	
						(*iter).collision = false;	
					
				}
			}		  
		 }			
	   }
	}
}


bool ready_to_find_wayout(map <int,int> mapInfo,int dstVex)
{
 	 bool flag = true;
 	 if  (dstVex != demandVex.dstvex) return flag;
 	 for (vector<int>::iterator iter = demandVex.demand.begin(); iter < demandVex.demand.end(); iter++)
 	 {
	  	 if(mapInfo[(*iter)] != 1) flag = false;
     }

     return flag;
}


static int OUTFLAG  = false;

bool find_possible_path(map <int,int> &Map,int startVertex ,int &VertexNum,const int VertexTotal,vector<path_list_t> &vec_list)
{	 
	 static int milisec  = clock();
      
	 if(clock() - milisec > OUT_MINISEC)  return false;	 
	 	 
	 srand((unsigned int)clock());                         //init seed	
     vector <DemandVexInfo> vec = map2Vector[startVertex]; //find start vector
     int size = vec.size(); 
     
     VertexNum++;
     if(size == 0) 
	 {	  
	  	OUTFLAG = true;
	  	cout << "error"<<endl;
	  	goto OUT;
	 }	 
     		   //added when pass a demanded points
     		   //pathList.push_back(startVertex);      	 
	 do {	   								 
	  		int offset = rand() % size; //init a random value 
		    if(randmomMap[startVertex][offset] == RESET)
		   {
				 randmomMap[startVertex][offset] = SET;
				 DemandVexInfo Info = vec[offset];
				 if( Map[startVertex]) continue;

				  Map[startVertex] = 1;
				
			     if ( false == ready_to_find_wayout(Map,Info.node.back().vertex) ) 
				 {	
				     Map[startVertex] = 0;
					 continue;
				 }	

				 if(false == add_path_in_list(Info,vec_list,VertexNum,VertexTotal))	 
				 {
				 	Map[startVertex] = 0;
				 	continue;
				 } 		
				 
				 if(Info.dstvex == demandVex.dstvex)
				 {
			          
			          cout << Info.dstvex << "  "<< demandVex.dstvex <<endl;
			          cout << VertexNum << endl;
			          cout << VertexTotal << endl;
			          OUTFLAG = true;
			          Map[startVertex] = 0;  //cancel make vertex
			          goto OUT;			          
				 }

				 //pruning 
				 //if(VertexNum >= 15 && pathList.size() / VertexNum > 2)  
				 //{
				 // 		vertexs_out_map(Map,Info,pathList); 
				 // 		goto OUT;	  
			     //} 
				  
				 //if((miniweight + Info.weight) >= weight_t) 
				 // {
				 //		vertexs_out_map(Map,Info,pathList); // already passed by other points 
				 //		continue;
				 //}
				 
				// miniweight += Info.weight; 

				// if(true == pass_all_demand_vertex(Map,Info.dstvex))
				// {
				//      if(Info.dstvex == demandVex.dstvex) 
				//	  {
				//	   		if(miniweight < weight_t) 
				//   			{
				//	 			  weight_t = miniweight;
				//	              FinalList = pathList;	
				//	        }		
			    //      }			 
				// }
				
		         find_possible_path(Map,Info.dstvex,VertexNum,VertexTotal,vec_list); 
		         Map[startVertex] = 0;  //cancel vertex
				 	         
		         if (true == OUTFLAG) goto OUT;                 
		         sub_path_in_list(Info,vec_list);
				      
		 		 //vertexs_out_map(Map,Info,pathList);	 		
		 		 //miniweight -= Info.weight;	 
		  }
	   	 								 
	 } while(randmom_map_all_set(randmomMap[startVertex],size) == false);
	 	 	
OUT:
	 clear_randmom_map(randmomMap[startVertex],size); 
	 VertexNum--;                            //if not match we should just change another route 
	 //Map[startVertex] = 0;	 
	 return OUTFLAG;
}



void set_connected_map(int src,int dst)
{
	connected_map[src][dst] = 1;
}

bool connected_in_map(int src,int dst)
{
	return (connected_map[src][dst] == 1) ? true: false;
}

static void  _swap(int target[],int length,int index,int value)
{
	int i,temp;
	for (i = 0;i < length;i++)
	{
		if (target[i] == value) break;
	}
	temp      = target[index];
	target[index] = target[i];
	target[i] = temp;
}
//Generate a posible route closest[][]
//  caculate 5!
bool permutation(int vertex[],int length,int endVertex)
{
	int *p = vertex+1;
	int len= length - 1;
	sort(p,p+len);	
	
	do{
		 for(int i = 0;i < len;i++)
		 {
		 	if (i == 0) 
			 {
			 	if (false == connected_in_map(vertex[0],p[0]))
				 {
				 	break;
				 } 
			 } 
			 else if( i != len - 1)
			 {	
			     if (false == connected_in_map(vertex[i-1],p[i]))
				 {
				 	break;
				 } 		 	
			 }
			 else
			 {
			 	if( true == connected_in_map(p[i],endVertex) && true == connected_in_map(p[i-1],p[i])) return  true;
			 }
		 }
	// for(int i = 0; i < length;i++)
	//cout << vertex[i] << "--";
	// cout << endl;
	}while(next_permutation(p,p+len));

	return false;
}
// abstract as a  recursion problem
bool combination_and_permutation(int vertex[],int length,int startVertex,int endVertex)
{	
	for (int i = 0;i < length;i++)
	{
rebegin: 
		srand(clock());
		if ( length - i <= 8 ) 
		{
		 	return permutation(&vertex[i],length-i,endVertex);//
		}
		else 
		{
		    int j = i;	
		 	do
		 	{
		 		j += 1;
		 		if(j == length) break;
			} while (false == connected_in_map(vertex[i],vertex[j]));
			
			if (j != length)
			{
				_swap(vertex,length,i+1,vertex[j]); //find out a close point then swap it
			}
			else
			{
				if(vertex[i] != startVertex) 
				{
					int index = rand() % (length - i);
					_swap(vertex,length,i,vertex[i+index]);
					
					goto  rebegin;
				}
				else
				{
					cout << "error happened:i="<<i<<endl;
					return false;
				}
			}					
		}
	}	
}
//to do when find a possible route vector <DemandVexInfo> vec = map2Vector[startVertex]; 

void record_routes_weight(map <int,vector <NodeInfo> > Map,vector <int> &routes_w, vector <NodeInfo> node,vector<int> vertex,int node_cnt)
{
	 if(node_cnt < 1) return;
	 
	 for(int i = 0;i <= node_cnt;i++) 
	 {
		 vector	<NodeInfo> Info = Map[vertex[i]]; 	
	  	for(vector<NodeInfo>::iterator iter = node.begin(); iter < node.end();iter++)
	  	{
		 
	 		for(vector<NodeInfo>::iterator it = Info.begin(); it < Info.end();it++)
	 		{
			   if((*iter).vertex == (*it).vertex) routes_w[i]++;
			}
		}	 
	 }
}

// record collision when collision happend

vector <int> find_path2record_collision(vector<int> vertex,int endVertex,map <int,vector <NodeInfo> > & Map,vector <NodeInfo> &Path)
{
	int size = vertex.size();
	vector <NodeInfo> node;
	vector <int> routes_w(size,0);
	vector <DemandVexInfo> vec;
	for( int i = 0;i < size;i++)
	{
		vec = map2Vector[vertex[i]];		
		for (std::vector<DemandVexInfo>::iterator iter = vec.begin(); iter < vec.end();iter++)
		{
			node = (*iter).node;
			int nodeback = (node.back()).vertex;
			if(i != size-1 )
			{
				if(nodeback == vertex[i+1])
				{
				 	Map[vertex[i]] = node;
				 	record_routes_weight(Map,routes_w,node,vertex,i); //todo:
				 	Path.insert(Path.end(),node.begin(),node.end());
				} 	
			}			
			else
			{
				if(nodeback == endVertex)   
				{
					Map[vertex[i]] = node;
				    record_routes_weight(Map,routes_w,node,vertex,i); //todo:
				    Path.insert(Path.end(),node.begin(),node.end());
				}
			}
		}
	}
	return routes_w;	
}

vector <int> generate_possible_route(vector<int> vertex,vector<int> &vertex_new,int endVertex,vector <NodeInfo> &Path)
{
	int offset;
	int startVertex = vertex[0];
	int length      = vertex.size();
    bool  result    = true;
    int          vertex_t[length];
    vector <int> collision; 
    map <int,vector <NodeInfo> > Map;
    
again:
	   
	while(vertex.size()) //init seed
	{	
		srand(clock());  
		offset = rand() % vertex.size();
		vertex_new.push_back(vertex[offset]);
		std::vector<int>::iterator iter = vertex.begin() + offset;
		vertex.erase(iter);
	}
	
	for (int i = 0; i < vertex_new.size();i++)
	    vertex_t[i] = vertex_new[i];
	_swap(vertex_t,length,0,startVertex);//judge first one is src point remain in the first pos 	
	
    //for (int i = 0; i < length;i++)
	//    cout << vertex_t[i] << "--";
	result = combination_and_permutation(vertex_t,length,startVertex,endVertex);//find the src points
	for (int i = 0; i < vertex_new.size();i++)
	    vertex_new[i] = vertex_t[i];  
	
	if (result == true) 
	{
		cout << "result:true";
	}
	if (result == false) goto again; //for(;;) while get a possible combination	
	//vector<int> vertex,int endVertex,int map <int,vector <NodeInfo> > &Map
	collision = find_path2record_collision(vertex_new,endVertex,Map,Path);
	return collision;		
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
  	 }else if(G->numofVex > 80 && G->numofVex <=200)
	 {
	 DynamicChangeNode = 10; 
	 
  	 } else
  	 {
	    DynamicChangeNode = 8; 	   
     }
}


void find_closest_points(Graph *G,string &outStr)
{
 	 arcnode *node;
 	 int i,j; 
 	 map <int,int > MapTemp;
	 int depth = 0,miniweight = 0;
	 list <int>myList;
 	 
     for(i = 0;i < DemandVexSize;i++)
     {
     	   for(j = 0; j < 600;j++)
     	   		 visit[i][j] = 0;
	 }
	 
     for(i = 0;i < DemandVexSize;i++)
     	   for(j = 0;j < 20;j++)
     	   		 closest[i][j] = 0;  
     	   		
	 set_dynamic_notes_num(G);
	 		 
     for(i = 0;i < DemandVexSize;i++)
     {
 	     init_tree_node(treeNode[i],demandVex.demand[i]);
   		 BFS(G,demandVex.demand[i],visit[i],closest[i],&treeNode[i]); 		 
  	 }  
	  	 
  	 for(i = 0;i < DemandVexSize;i++)
  	 {
	  	   vector<NodeInfo> path;
	  	   vector<DemandVexInfo> paths;
	  	   
	  	   get_path_info(&treeNode[i],path,paths,G);
	  	   pathInfo.push_back(paths);
     }
  
     for(i = 0;i < DemandVexSize;i++)
     { 	  
	  	  for(std::vector<DemandVexInfo>::iterator iter = pathInfo[i].begin(); iter < pathInfo[i].end();iter++) 
	  	  {     // DemandVexInfo.node[0].vertex not the begin vertex  the last one was 
				 int weight = 0,dstvex = 0;  
				 for(vector<NodeInfo>::iterator it = (*iter).node.begin(); it < (*iter).node.end(); it++)
				 {
		   		       dstvex  = (*it).vertex; 
		   		       weight += (*it).weight;	       
                 }               
 			     (*iter).srcvex = demandVex.demand[i];
 			     
 			     //cout <<"src1:"<<(*iter).srcvex<<"src2"<<(*iter).node[0].vertex<<endl; 
  				 (*iter).dstvex = dstvex;
  				 (*iter).weight = weight; 	
				 set_connected_map((*iter).srcvex,(*iter).dstvex);			
		        	 	         
	      }  
   	 } 	 	
	 cout <<"result:"<<endl;   
	 map_init(pathInfo,map2Vector,demandVex);  
	 
#if 0	   	 
   	 //vector <int> collision;
   	 //vector <NodeInfo> Path_t;
   	 //vector <int> vertex_t;	 
   	 //collision = generate_possible_route(demandVex.demand,vertex_t,demandVex.dstvex,Path_t);
   	 //(map <int,int> Map,int startVertex ,int &VertexNum,const int VertexTotal,vector<path_list_t> &vec_list)

   	 vector<path_list_t> vec_list;
	 find_possible_path(MapTemp,demandVex.srcvex,depth,DemandVexSize,vec_list);
	 for (i = 0; i < vec_list.size();i++)
	     for(vector <int>::iterator iter= vec_list[i].list.begin(); iter < vec_list[i].list.end();iter++)
	      	cout << *iter << "--";
	      	
	cout << "end";
	while(1);

#else
   	
     list<int> FinalList; 	 
   	 //int weight = DFS(MapTemp,demandVex.srcvex,depth,myList,FinalList,miniweight,G);  
	 int weight = main_loop(demandVex.srcvex,FinalList,G);	
	 cout << "minest weight:"<<weight<<" steps: "<< FinalList.size()+1 <<" average w:" << average_weight<<"demand.size:" << DemandVexSize <<endl; 
	 	 
	 if(weight != INFINITY)
	 {
		 FinalList.push_back(demandVex.dstvex);
		 list <int>::iterator iter_t;
		 for(list <int>::iterator iter = FinalList.begin();iter != FinalList.end();iter++)
		 {
		  		  string str;	  		  
		  		  if(iter != FinalList.begin())
		  		  {	   		       
		  		   int num = get_edge_number(G,*(iter_t),*(iter));
		  		   int2str(num,str);
		  		   outStr+=str+'|';
				  }
				  iter_t = iter;
	  	 }
	  	 outStr = outStr.substr(0,outStr.size()-1);
	  	 refresh_write_buff(outStr);
	 }	
#endif 

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
	return INFINITY;
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
	return INFINITY; 	
}

//dijkstra(G,demandVex.srcvex ,Dis_dijkstra,Path_dijkstra,Queue_dijktra) 

typedef pair<int,int> pair_t;

void dijkstra(Graph *G,int src,int d[],int p[],int done[])
{
 priority_queue<pair_t,vector<pair_t>,greater <pair_t> > q;
 q.push(make_pair(d[src],src));
 for(int i = 0;i < 1000;i++) d[i] = (i == src ? 0:INFINITY);
 for(int i = 0;i < 1000;i++) p[i] = NIL;
 for(int i = 0;i < 1000;i++) done[i] = 0; 
 while(!q.empty())
 {
    pair_t u = q.top();
    q.pop();
    int ver1 = u.second;     // ver current node number
	if(done[ver1]) continue; // already caculate 
	done[ver1] = 1;
    arcnode *next = G->v[ver1].next;
	while (next != NULL)
	{
		int ver2 = next->vertex;
		int w    = get_edge_weight(G,ver1,ver2);
		if(d[ver2] > d[ver1] + w)
		{
  		   d[ver2] = d[ver1] + w;
  		   p[ver2] = ver1;  // save path info
  		   q.push(make_pair(d[ver2],ver2));
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


#include <iostream>
#include "ga.h"
using namespace std;

#define POP_SIZE  20

// void replace_shortest_route(vector <PathsVector> GlobalVec,list<int> &pathList,Graph *G,map <int,int> &mapInfo)
//typedef struct pathvector
//{
//	int src;
//	int dst;    // next destination
//	int weight;
//	vector <int> path;
//}PathsVector;
//optimization using dfs
void exchange_section(vector <PathsVector> &vect,Graph *G,int r1,int n,int m,bool limit);


extern map <int,int> OutDegreeOne;
extern map <int,int> InDegreeOne;

void mutate(vector <PathsVector> &vect,Graph *G,bool limit)
{
   int size = vect.size() - 2; // src and dst vertex can't change 
   int r1,r2,r3;
   int prev1,prev2,next1,next2,now1,now2,_now2;
   bool connect = false;
   bool change_section = false;
   int m,n;
   
   srand((unsigned int)clock());
   do {
   	    
		r1 = 1 + rand() % size;
   		r2 = 1 + rand() % size;

   		if (r1 > r2) { int t = r1; r1 = r2; r2 = t;}
   		/////////////////////////////////////////////////
  #if 0		
   		// vertex has only one indegree or one outdegree
		if ( (OutDegreeOne[vect[r2].src] || InDegreeOne[vect[r2].src]) && r2 - r1 > 2 ) // to do:?
 		{	
			 m = n = r2;
			 
			if (OutDegreeOne[vect[r2].src])
			{		
				m = r2 + 1;
				while(OutDegreeOne[vect[m].src] && m < size) m++;
			}
			
			if (InDegreeOne[vect[r2].src])
			{
				n = r2 - 1;
				while(InDegreeOne[vect[n].src] && n > r1 + 2) n--;	 // to make sure has one more vertex between then 
			}	
			
			now1  = vect[r1].src  ; now2  = vect[n].src;
	   		prev1 = vect[r1-1].src; next1 = vect[r1+1].src;
	   		prev2 = vect[n-1].src ; next2 = vect[m+1].src;	
			_now2 = vect[m].src;
	   		if (connected_in_map(prev1,now2) && connected_in_map(_now2,next1)
			 && connected_in_map(prev2,now1) && connected_in_map(now1,next2) )
			{
				connect = true;
				change_section = true;
			} else
			{
				connect = false;
				change_section = false;
			}		   									
		}
		/////////////////////////////////////////////////////////////////////////// 
		else
		#endif
		
		{
	   		if((r2 - r1 ) == 1)
	   		{
	   			now1 = vect[r1].src ; now2 = vect[r2].src;
	   			prev1 = vect[r1-1].src;
	   			next2 = vect[r2+1].src;
	   			if(connected_in_map(prev1,now2) && connected_in_map(now2,now1) && connected_in_map(now1,next2))
	   			{
	   				connect = true;
				} else
				{
					connect = false;
				} 			
			}
	   		
	   		else
	   		{ 
				now1  = vect[r1].src ; now2 = vect[r2].src;
		   		prev1 = vect[r1-1].src;next1 = vect[r1+1].src;
		   		prev2 = vect[r2-1].src;next2 = vect[r2+1].src;
		   	    if(connected_in_map(prev1,now2) && connected_in_map(now2,next1)
				&& connected_in_map(prev2,now1) && connected_in_map(now1,next2) )
				{
					connect = true;
				} else
				{
					connect = false;
				}
			}					
		}	
		
   } while (r1 == r2 || connect == false);
   
 	if (true == change_section) 
   {
 		exchange_section(vect,G,r1,n,m,limit);
   	    cout << "change sec" <<endl;
   }
 	else 
   {
   		exchange_two_demand_points(vect,G,r1,r2,limit);
   } 
  //r3 = rand();
}


extern int Dis_dijkstra [];
extern int Path_dijkstra[];
extern int Done_dijktra []; 

extern double average_weight;
extern int mutate_cnt;
// need r1 < r2
// limited way used to mutation
// unlimited way used to generate a population of the city 

void exchange_section(vector <PathsVector> &vect,Graph *G,int r1,int n,int m,bool limit)
{
	vector <PathsVector> p  = vect;	
	vector <NodeInfo> Info;
	map <int,int> Map;
	map <int,int> mapInfo;
	int a,b,prev1,next1,prev2,next2;
	int i;
	int w_now = 0,w_pre = 0;
	
	//cout << n <<" "<< m << "n:m" <<endl;
	
	w_pre = p[r1-1].weight + p[r1].weight + p[n-1].weight + p[m].weight;
	
	a = p[n].src; b = p[m].src;
	
	prev2 = p[n-1].src;
	next2 = p[m+1].src;
	
	
	//p[n-1].src  p[m+1].src
	prev1 = p[r1-1].src;  next1 = p[r1+1].src;
	
	//cout << "#" << a <<  "#" <<  b <<  "#" << prev1 <<  "#" <<  next1 <<  "#" << prev2 <<  "#" << next2 <<endl;
	for (i = n;i <= m;i++)
	{
	     Map[p[i].src] = 1;	
	}
	
	for (i = 0;i < p.size();i++)
	{
		mapInfo[p[i].src] = 1;
		mapInfo[p[i].dst] = 1;
		if ( i != r1 && i != r1-1 && i != n-1 && i != m) // only two pass eliminate
		{
    		for(int j = 0; j < p[i].path.size();j++)
    		{
    			mapInfo[p[i].path[j]] = 1;
			}
		}
	}	
	
	vector<PathsVector>::iterator it;  // erase vertex paths inorder 
	for(it = p.begin(); it < p.end();it ++)
	{
		while(Map[(*it).src] && (*it).src != b) 
		{
			//cout << "erase:" << (*it).src << endl;
			it = p.erase(it);	
		}
	}
	
 	for(it = p.begin(); it < p.end();it++ )
 	{
 		if ((*it).src == vect[n-1].src) break;
	}
	
	(*it).dst  = vect[r1].src;
		
	mapInfo[(*it).src] = 0;	mapInfo[(*it).dst] = 0;		
	dijkstra(G,(*it).src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
	record_route((*it).src,(*it).dst,Path_dijkstra,Info,G);
	mapInfo[(*it).src] = 1;	mapInfo[(*it).dst] = 1;	
		
	if (Path_dijkstra[(*it).dst] != NIL)
	{
		(*it).weight = Dis_dijkstra[(*it).dst];
		w_now += (*it).weight;
		(*it).path.clear();
		for(int x = 1; x < Info.size(); x++)
		{
			mapInfo[Info[x].vertex] = 1;
			(*it).path.push_back(Info[x].vertex);
		}
					
	} else
	{
		return; 
	}
	
	// m is invalid in p from now on
	
	for(it = p.begin(); it < p.end();it ++)
	{
		if( b == (*it).src) break;
	}
	
	(*it).src = vect[r1].src;
	(*it).dst = vect[m+1].src;
	
	Info.clear(); 			 // clear Info
	mapInfo[(*it).src] = 0;	mapInfo[(*it).dst] = 0;		
	dijkstra(G,(*it).src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
	record_route((*it).src,(*it).dst,Path_dijkstra,Info,G);
	mapInfo[(*it).src] = 1;	mapInfo[(*it).dst] = 1;	
		
	if (Path_dijkstra[(*it).dst] != NIL)
	{
		(*it).weight = Dis_dijkstra[(*it).dst];
		w_now += (*it).weight;
		(*it).path.clear();
		for(int x = 1; x < Info.size(); x++)
		{
			mapInfo[Info[x].vertex] = 1;
			(*it).path.push_back(Info[x].vertex);
		}
					
	} else
	{
		return; 
	}
	
	for(it = p.begin(); it < p.end(); it++)
	{
			if((*it).src == vect[r1].src) 
			{
				p.erase(it); break;  // it point to next iterator 
			}
	}
	
	//for(it = p.begin(); it < p.end(); it++)
	//{
	//		if((*it).src == vect[r1+1].src) break;
	//}
	//if( (*it).src == vect[r1+1].src) cout << "hehe"<<endl;
	
	for (i = n; i <= m;i++) 
	{
		for(it = p.begin(); it < p.end(); it++)
		{
				if((*it).src == vect[r1+1].src) break;
		}
		p.insert(it,vect[i]);
	}	
	
	for(it = p.begin(); it < p.end(); it++)
	{
		if((*it).src == prev1) break;
	}	
	
	Info.clear();
	(*it).dst = a ;
	mapInfo[(*it).src] = 0;	mapInfo[(*it).dst] = 0;		
	dijkstra(G,(*it).src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
	record_route((*it).src,(*it).dst,Path_dijkstra,Info,G);
	mapInfo[(*it).src] = 1;	mapInfo[(*it).dst] = 1;	
		
	if (Path_dijkstra[(*it).dst] != NIL)
	{
		(*it).weight = Dis_dijkstra[(*it).dst];
		w_now += (*it).weight;
		(*it).path.clear();
		for(int x = 1; x < Info.size(); x++)
		{
			mapInfo[Info[x].vertex] = 1;
			(*it).path.push_back(Info[x].vertex);
		}
					
	} else
	{
		return; 
	}
	
	for(it = p.begin(); it < p.end(); it++)
	{
		if((*it).src == b)  break;
	}	
	Info.clear();
    	(*it).dst = next1;
	mapInfo[(*it).src] = 0;	mapInfo[(*it).dst] = 0;		
	dijkstra(G,(*it).src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
	record_route((*it).src,(*it).dst,Path_dijkstra,Info,G);
	mapInfo[(*it).src] = 1;	mapInfo[(*it).dst] = 1;	

	if (Path_dijkstra[(*it).dst] != NIL)
	{
		(*it).weight = Dis_dijkstra[(*it).dst];
		w_now += (*it).weight;
		(*it).path.clear();
		for(int x = 1; x < Info.size(); x++)
		{
			mapInfo[Info[x].vertex] = 1;
			(*it).path.push_back(Info[x].vertex);
		}					
	} else
	{
		return; // return 
	}
	
	// change  value at last
	if (w_now < w_pre + 4)  
	{
		vect = p;
		//cout << "change section" <<endl;
	}

}		

// a b c d e f g h i j k l m n
void exchange_two_demand_points(vector <PathsVector> &GlobalVec,Graph *G,int r1,int r2,bool limit)
{
	map <int,int> mapInfo;
	vector <vector <NodeInfo> > Info(4);
	bool mutate_flag = false;
	int w[4] = {0};
	int step[4] = {0};
	int step1 = 0,step2 = 0;
	int w1 = 0,w2 = 0;
	int cnt;
	int ver[4][2];
	int prev1 = r1 - 1;
	int prev2 = r2 - 1;
	int next1 = r1 + 1;
	int next2 = r2 + 1;
	
	if((r2 - r1) != 1)
	{
		ver[0][0] = prev1;
		ver[0][1] = r2;
		ver[1][0] = r2;
		ver[1][1] = next1;
		ver[2][0] = prev2;
		ver[2][1] = r1;
		ver[3][0] = r1;
		ver[3][1] = next2;	
		cnt = 4;	
	} else	
	{
		ver[0][0] = prev1;
		ver[0][1] = r2;
		ver[1][0] = r2;
		ver[1][1] = r1;
		ver[2][0] = r1;
		ver[2][1] = next2;
		cnt = 3;
	}

	for(int i = 0;i < GlobalVec.size() ;i++)
	{
		mapInfo[GlobalVec[i].src] = 1;
		mapInfo[GlobalVec[i].dst] = 1;
		
		if (i != r1 && i != r2 && i != prev1 && i != prev2)
		{
	    	for(int j = 0; j < GlobalVec[i].path.size();j++)
	    	{
	    		mapInfo[GlobalVec[i].path[j]] = 1;
			}
		}
	}
		
	for (int i = 0; i < cnt;i++)
	{
		mapInfo[GlobalVec[ver[i][0]].src] = 0;	mapInfo[GlobalVec[ver[i][1]].src] = 0;		
		step2 += (GlobalVec[ver[i][0]].path.size() + 1); // caculate all the step 
		dijkstra(G,GlobalVec[ver[i][0]].src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
		record_route(GlobalVec[ver[i][0]].src,GlobalVec[ver[i][1]].src,Path_dijkstra,Info[i],G);
		
		mapInfo[GlobalVec[ver[i][0]].src] = 1;	mapInfo[GlobalVec[ver[i][1]].src] = 1;	
		if (Path_dijkstra[GlobalVec[ver[i][1]].src] != NIL)
		{
			w[i] = Dis_dijkstra[GlobalVec[ver[i][1]].src];
			w1 += w[i];
			step1 += Info[i].size();
			for(int n = 1; n < Info[i].size(); n++)
			{
				mapInfo[Info[i][n].vertex] = 1;
			}
						
		} else
		{
			return;
		}			
	}	
	
	if((r2 - r1) != 1)  w2 = GlobalVec[r1].weight + GlobalVec[r2].weight + GlobalVec[prev1].weight + GlobalVec[prev2].weight;
	else  		    w2 = GlobalVec[r1].weight + GlobalVec[r2].weight + GlobalVec[prev1].weight;
	 // mutate_cnt 
	 // about how to choose the the mutation it will 
	 //if (mutate_cnt > 0 && mutate_cnt < 200 && w2 > w1 + 5) mutate_flag = true;
	 //else if (mutate_cnt >= 200 && mutate_cnt < 400 && (w2 > w1 + 3 || w2 + (average_weight / 3) * step2 >= w1 + (average_weight / 3) * step1)) mutate_flag = true;	
	 //else if (mutate_cnt >= 400 && w2 > w1) mutate_flag = true;
	 
	 //if(w2 > w1 + 1 || w2 + (average_weight / 2) * step2 >= w1 + (average_weight / 2) * step1 ) mutate_flag = true;

#if 0	
	 if (mutate_cnt < 200)
	 {
		// && w2 - (average_weight / 1.0) * step2 >= w1 - (average_weight / 1.0) * step1
	 	if ( w2 > w1 + 5  ) mutate_flag = true;
	 }
	 else if (mutate_cnt < 500) //&& w2 - (average_weight / 2.0) * step2 >= w1 - (average_weight / 2.0) * step1
	 {
		if ( w2 > w1 + 3 || (w2 - (average_weight / 2) * step2 >= w1 - (average_weight / 2) * step1) ) mutate_flag = true;
	 }
	 
	 else if (w2 > w1 || (w2 + (average_weight / 2) * step2 >= w1 + (average_weight / 2) * step1) ) mutate_flag = true;  
	 
#endif

     if (w2 > w1 ) mutate_flag = true;
     
     //else if ((step2 > step1 + 3) && w2 + 5 > w1) {cout<<"@"<<endl; mutate_flag = true;}
     
	 if (true == mutate_flag || limit == false)
	 {
	 	///cout << "exchange" << endl;
		if((r2 - r1) != 1)
		{
			int t  = GlobalVec[r1].src;
			
			GlobalVec[prev1].path.clear();
			for(int n = 1; n < Info[0].size(); n++)
			{
				GlobalVec[prev1].path.push_back(Info[0][n].vertex);
			}
			GlobalVec[prev1].dst 	= GlobalVec[r2].src;
			GlobalVec[prev1].weight = w[0];
			
			GlobalVec[r1].path.clear();	
			for(int n = 1; n < Info[1].size(); n++)
			{
				GlobalVec[r1].path.push_back(Info[1][n].vertex);
			}
			GlobalVec[r1].src 	= GlobalVec[r2].src;
			GlobalVec[r1].weight 	= w[1];
			
			GlobalVec[prev2].path.clear();	
			for(int n = 1; n < Info[2].size(); n++)
			{
				GlobalVec[prev2].path.push_back(Info[2][n].vertex);
			}
			GlobalVec[prev2].dst 	= t;
			GlobalVec[prev2].weight = w[2];
			
				
			GlobalVec[r2].path.clear();
			for(int n = 1; n < Info[3].size(); n++)
			{
				GlobalVec[r2].path.push_back(Info[3][n].vertex);
			}
			GlobalVec[r2].src 	= t;
			GlobalVec[r2].weight 	= w[3];
						
		} else
		{
			int t  = GlobalVec[r1].src;
			
			GlobalVec[prev1].path.clear();
			for(int n = 1; n < Info[0].size(); n++)
			{
				GlobalVec[prev1].path.push_back(Info[0][n].vertex);
			}
			GlobalVec[prev1].dst 	= GlobalVec[r2].src;
			GlobalVec[prev1].weight = w[0];
			
			GlobalVec[r1].path.clear();	
			for(int n = 1; n < Info[1].size(); n++)
			{
				GlobalVec[r1].path.push_back(Info[1][n].vertex);
			}
			GlobalVec[r1].src    = GlobalVec[r2].src;
			GlobalVec[r1].dst    = t;
			GlobalVec[r1].weight = w[1];
			
			GlobalVec[r2].path.clear();	
			for(int n = 1; n < Info[2].size(); n++)
			{
				GlobalVec[r2].path.push_back(Info[2][n].vertex);
			}
			GlobalVec[r2].src    = t;
			GlobalVec[r2].weight = w[2];			
		}		
	 }
	 
}


void population(vector <PathsVector> &vect, vector <vector <PathsVector> > &pop,Graph *G)
{
	vector <PathsVector> vec_t;
	int cnt = 0;
	for(int i = 0;i < POP_SIZE;i++)
	{	
		vec_t = vect;
		while(cnt++ < 30)
		{
			mutate(vec_t,G,false);
		}
		pop.push_back(vec_t);
	}
}

// cross over the our current solution
#define CROSS_LEN  3
void crossover(vector <vector <PathsVector> > &pop ,Graph *G)
{
	int index1,index2;
	vector <NodeInfo> Info;
	int r1,r2,a,b;
	int i,j;
	int size = pop[0].size() - 2; // src and dst vertex can't change ;
	map <int,int> crossMap;
	map <int,int> mapInfo;
	vector <PathsVector> p1,p1_t,p2,p2_t;
	do{
		index1 = rand() % pop.size();
		index2 = rand() % pop.size();
	} while(index1 == index2);
	p1_t = p1 = pop[index1];
	p2_t = p2 = pop[index2];
 	do {
   	    r1 = 1 + rand() % size;
   		r2 = 1 + rand() % size;
   		if(r1 > r2) { int t = r1; r1 = r2; r2 = t;}
	}while(r1 == r2 || r2 > r1 + CROSS_LEN);
	
	a = p1[r1].src; b = p1[r2].src;
	for (i = r1;i <= r2;i++)
	{
	     crossMap[p1[i].src] = 1;	
	}
	
	for (i = 0;i < pop[0].size();i++)
	{
		mapInfo[p2[i].src] = 1;
		mapInfo[p2[i].dst] = 1;
		if (!crossMap[p2[i].src] && !crossMap[p2[i+1].src])
		{
    		for(int m = 0; m < p2[i].path.size();m++)
    		{
    			mapInfo[p2[i].path[m]] = 1;
			}
		}
	}
	
	// erase exsist vertex selected from p1 
	for (i = 1;i < pop[0].size();i++)
	{
		j = i;
		Info.clear(); // clear Info
		if(crossMap[p2[i].src])
		{
		 	do {
				vector<PathsVector>::iterator it;
				for(it = p2.begin(); it < p2.end(); it++)
				{
					if((*it).src == p2[j].src) 
					{
						it = p2.erase(it); break;
					}
				}
		    		crossMap[p2[j++].src] = 0;
	 	    }while(crossMap[p2[j].src]);
		} else 
		{
			continue;
		}
		
		p2[i-1].dst = p2[j].src; // point to src 
		mapInfo[p2[i-1].src] = 0;	mapInfo[p2[j].src] = 0;		
		dijkstra(G,p2[i-1].src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
		record_route(p2[i-1].src,p2[j].src,Path_dijkstra,Info,G);
		mapInfo[p2[i-1].src] = 1;	mapInfo[p2[j].src] = 1;	
		if (Path_dijkstra[p2[j].src] != NIL)
		{
			p2[i-1].weight = Dis_dijkstra[p2[j].src];
			for(int n = 1; n < Info.size(); n++)
			{
				mapInfo[Info[n].vertex] = 1;
				p2[i-1].path.push_back(Info[n].vertex);
			}					
		} else
		{
			return; // return 
		}
	}
	
	for (i = 0; i < p2.size();i++)
	{		
		if (connected_in_map(p2[i].src,a) && connected_in_map(b,p2[i+1].src))
		{
			p1[r2].dst = p2[i+1].src;
			vector<PathsVector>::iterator it;
			//for(it = p2.begin(); it < p2.end(); it++)
			//{
			//	if((*it).src == p2[i+1].src) break;
			//}			
			for (int n = r1; n <= r2;n++) // need to refind path from vertex r2
			{
				for(it = p2.begin(); it < p2.end(); it++)
				{
					if((*it).src == p2[i+1].src) break;
				}
				p2.insert(it,p1[n]);
				mapInfo[p1[n].src] = 1;mapInfo[p1[n].dst] = 1;
				if(n != r2)
				{
					for(int m = 0; m < p1[n].path.size();m++)
						mapInfo[p1[n].path[m]] = 1;
				}
			}			
			Info.clear();
			p2[i].dst = a ;// point to src 
			mapInfo[p2[i].src] = 0;	mapInfo[p2[i].dst] = 0;		
			dijkstra(G,p2[i].src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
			record_route(p2[i].src,p2[i].dst,Path_dijkstra,Info,G);
			mapInfo[p2[i].src] = 1;	mapInfo[p2[i].dst] = 1;	
			if (Path_dijkstra[p2[i].dst] != NIL)
			{		
				p2[i].weight = Dis_dijkstra[p2[i].dst];
				for(int n = 1; n < Info.size(); n++)
				{
					mapInfo[Info[n].vertex] = 1;
					p2[i].path.push_back(Info[n].vertex);
				}					
			} else
			{
				return; // return 
			}
					
			for(it = p2.begin(); it < p2.end(); it++)
			{
				if((*it).src == b) break;
			}
			if(it == p2.end()) cout << "error";
			
			Info.clear();

			mapInfo[(*it).src] = 0;	mapInfo[(*it).dst] = 0;		
			dijkstra(G,(*it).src,Dis_dijkstra,Path_dijkstra,Done_dijktra,mapInfo);
			record_route((*it).src,(*it).dst,Path_dijkstra,Info,G);
			mapInfo[(*it).src] = 1;	mapInfo[(*it).dst] = 1;	

			if (Path_dijkstra[(*it).dst] != NIL)
			{
				(*it).weight = Dis_dijkstra[(*it).dst];
				for(int n = 1; n < Info.size(); n++)
				{
					mapInfo[Info[n].vertex] = 1;
					(*it).path.push_back(Info[n].vertex);
				}					
			} else
			{
				return; // return 
			}
			break;	
		}	
			
	}
	pop.push_back(p2);
}

void replace_npoints_in_dij(vector <PathsVector> &vect)
{
	//int r =
}
void mutation(vector <vector <PathsVector> > &pop ,Graph *G)
{
	for(int i = 0; i < pop.size();i++)
	{
		mutate(pop[i],G,true);
	}
}

void elitist(vector <vector <PathsVector> > &pop)
{
	while(pop.size() > 20)
	{
		int max = 0,min = 0,w = 0,min_w = 10000000, max_w = 0;
		vector <vector <PathsVector> >::iterator it = pop.begin();
	    for(int i = 0; i < pop.size();i++)
		{
			for(int j = 0; j < pop[i].size();j++)
			{
				w += pop[i][j].weight;
			}
			if(w < min_w)
			{
				min_w = w;
				min   = i;
			}
	
			if(w > max_w)
			{
				max_w = w;
				max   = i;
			}
		}
		pop.erase(it + max);
	}	
}

static vector <PathsVector> _get_best_child(vector <vector <PathsVector> > &pop)
{
	int min = 0,w = 0,min_w = 10000000;
	vector <vector <PathsVector> >::iterator it = pop.begin();
    for(int i = 0; i < pop.size();i++)
	{
		for(int j = 0; j < pop[i].size();j++)
		{
			w += pop[i][j].weight;
		}
		if(w < min_w)
		{
			min_w = w;
			min   = i;
		}
	}
	return pop[min];	
}

void ga(vector <PathsVector> &vect, vector <vector <PathsVector> > &pop,Graph *G)
{	int count = 0;
	int cross_cnt ;
	
	population(vect,pop,G);
	
	while(count++ < MAX_GEN)
	{
		cross_cnt = 0;
		while(cross_cnt++ < 4) crossover(pop,G);
		mutation(pop,G);
		elitist(pop);
	} 
	vect = _get_best_child(pop);
}

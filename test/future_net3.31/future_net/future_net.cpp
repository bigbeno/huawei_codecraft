#include "route.h"
#include "lib_io.h"
#include "lib_time.h"
#include "stdio.h"
#include <string>

using namespace std;

Graph  graph;
extern DemandVertex demandVex;
int main(int argc, char *argv[])
{
    print_time("Begin");
    char *topo[5000];
    int edge_num;

    char *demand;
    int demand_num;
	string OutString;
	
    char *topo_file = argv[1];

    edge_num = read_file(topo, 5000, topo_file);

    if (edge_num == 0)
    {
        printf("Please input valid topo file.\n");
        return -1;
    }
    char *demand_file = argv[2];

    demand_num = read_file(&demand, 1, demand_file);
    if (demand_num != 1)
    {
        printf("Please input valid demand file.\n");
        return -1;
    }
    graph_init(&graph,edge_num);
    build_graph(&graph,topo,demand,demandVex);
    print_graph(&graph);
    find_closest_points(&graph,OutString);

    search_route(topo, edge_num, demand);

    char *result_file = argv[3];
    write_result(result_file);
    destory_graph(&graph);
    release_buff(topo, edge_num);
    release_buff(&demand, 1);

    print_time("End");
    while(1);
    return 0;
}


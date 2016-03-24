#ifndef __ROUTE_H__
#define __ROUTE_H__
#include "lib_record.h"
#include <stdio.h>

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <unordered_map>
#include <set>

#include "graph.h"
#include "dijkstra.h"


bool try_search(Graph, int node_num, char *demand, int keep_max, int k);
void search_route(char *graph[5000], int edge_num, char *condition);
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);
int handle_graph(char **topo, int edge_num, Graph &graph);
int read_demand(char *demand, int &start, int &end, std::vector<int> &demand_nodes);

#endif

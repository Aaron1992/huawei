#ifndef FUTURE_DIJKSTRA_
#define FUTURE_DIJKSTRA_
#include <iostream>
#include <vector>
#include <string>
#include <list>

#include <limits> // for numeric_limits
#include <functional> 
#include <queue>
#include <utility> // for pair
#include <algorithm>
#include <iterator>
#include "graph.h"


void DijkstraComputePaths(vertex_t source,
	const adjacency_list_t &adjacency_list,
	std::vector<weight_t> &min_distance,
	std::vector<vertex_t> &previous);


std::vector<vertex_t> DijkstraGetShortestPathTo(
	vertex_t vertex, const std::vector<vertex_t> &previous);
std::vector<vertex_t> shortest_path(Graph &graph, vertex_t source, vertex_t sink);

int path_length(Graph &graph, std::vector<vertex_t> path);
std::vector<std::vector<vertex_t>> k_shortest_path(Graph &graph, vertex_t source, vertex_t sink, int k_number);
#endif
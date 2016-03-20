#ifndef FUTURE_GRAPH_
#define FUTURE_GRAPH_
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <string>
#include <limits>
#define INF 10000000
extern int NODE_NUM;
typedef int vertex_t;
typedef double weight_t;
struct neighbor {
	int id;
	vertex_t target;
	weight_t weight;
	neighbor(int arg_id, vertex_t arg_target, weight_t arg_weight)
		: id(arg_id), target(arg_target), weight(arg_weight){ }
};

typedef std::vector<std::vector<neighbor> > adjacency_list_t;
typedef std::pair<weight_t, vertex_t> weight_vertex_pair_t;
const weight_t max_weight = std::numeric_limits<double>::infinity();


class ShortPath
{
public:
	int cost;
	vertex_t start;
	unsigned int bitmarker[19];
	std::vector<int> order;
	ShortPath();
	ShortPath(int arg_cost, std::vector<vertex_t> arg_path);
	friend ShortPath operator+(const ShortPath &c1, const ShortPath &c2);
};

//记录下各点最短距离经过的边
struct ShortRoute{
	vertex_t start;
	vertex_t end;
	std::vector<vertex_t> route;
};

class PathCompare{
	bool reverse;
public:
	PathCompare(const bool& revparam = false)
	{
		reverse = revparam;
	}
	bool operator() (const ShortPath & lhs, const ShortPath&rhs) const
	{
		if (reverse) return (lhs.cost<rhs.cost);
		else return (lhs.cost>rhs.cost);
	}
};

class Graph{
private:
	int link_num_;
	int node_num_;
public:
	Graph();
	Graph(int node_num);
	adjacency_list_t adjacency_list;
	void add_link(int id, int source_id, int dest_id, int cost);
	int size();
};

bool check(ShortPath path_frag, ShortPath long_path);
bool confilct(ShortPath c1, ShortPath c2);
int count(ShortPath c, const unsigned int demand_vector[]);


#endif

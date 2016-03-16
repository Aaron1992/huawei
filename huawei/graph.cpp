#include "graph.h"

int NODE_NUM = 0;

Graph::Graph(int node_num){
	node_num_ = node_num;
	adjacency_list.resize(node_num);
}

Graph::Graph(){

}

void Graph::add_link(int id, int source_id, int dest_id, int cost){
	adjacency_list[source_id].push_back(neighbor(id, dest_id, cost));
}

ShortPath::ShortPath(){
	cost = 0;
	for (auto &v : bitmarker) v = 0;
}

ShortPath::ShortPath(int arg_cost, std::vector<vertex_t> arg_path){
	cost = arg_cost;
	path = arg_path;
	for (unsigned int &b : bitmarker) b = 0;
	for (int i = 1; i < arg_path.size(); i++){
		bitmarker[arg_path[i] / 32] += (1 << (arg_path[i] % 32));
	}
};

ShortPath operator+(const ShortPath &c1, const ShortPath &c2){
	ShortPath p;
	p.cost = c1.cost + c2.cost;


	for (int i = 0; i <= NODE_NUM / 32; i++){
		p.bitmarker[i] = c1.bitmarker[i] | c2.bitmarker[i];
	}
	p.path.reserve(c1.path.size() + c2.path.size()-1);
	p.path.insert(p.path.end(), c1.path.begin(), c1.path.end());
	p.path.insert(p.path.end(), c2.path.begin()+1, c2.path.end());
	return p;
}

bool confilct(ShortPath c1, ShortPath c2) 	{
	if (1 << (c1.path[0] % 32)&c2.bitmarker[c1.path[0] / 32]) return true;
	for (int i = 0; i <= NODE_NUM / 32; i++){
		if ((c1.bitmarker[i] & c2.bitmarker[i]) != 0)
			return true;
	}
	return false;
}

int count(ShortPath c, const unsigned int demand_vector[], int node_num){
	unsigned int sign_num = 0;
	unsigned int x;
	int count;
	for (int i = 0; i <= node_num / 32; i++){
		x = c.bitmarker[i] & demand_vector[i];
		for (count = 0; x; count++)
			x &= x - 1;
		sign_num += count;
	}
	return sign_num;
}
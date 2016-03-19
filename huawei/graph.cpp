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
	if (arg_cost < 0){
		cost = 1000000;
	}
	else{
		cost = arg_cost;
	}
	start = arg_path[0];
	order.push_back(start);
	for (unsigned int &b : bitmarker) b = 0;
	for (int i = 1; i < arg_path.size(); i++){
		bitmarker[arg_path[i] / 32] = bitmarker[arg_path[i] / 32] | (1 << (arg_path[i] % 32));
	}
};

ShortPath operator+(const ShortPath &c1, const ShortPath &c2){
	ShortPath p;
	p.cost = c1.cost + c2.cost;

	for (int i = 0; i <= NODE_NUM / 32; i++){
		p.bitmarker[i] = c1.bitmarker[i] | c2.bitmarker[i];
	}
	p.start = c1.start;
	p.order.reserve(c1.order.size() + c2.order.size()); // preallocate memory
	p.order.insert(p.order.end(), c1.order.begin(), c1.order.end());
	p.order.insert(p.order.end(), c2.order.begin(), c2.order.end());
	return p;
}

bool confilct(ShortPath c1, ShortPath c2) 	{
	if (1 << (c1.start % 32)&c2.bitmarker[c1.start / 32]) return true;
	for (int i = 0; i <= NODE_NUM / 32; i++){
		if ((c1.bitmarker[i] & c2.bitmarker[i]) != 0)
			return true;
	}
	return false;
}

//检查路径中有多少点，既位向量中有多少1
int count_node(ShortPath p){
	unsigned int sign_num = 0;
	unsigned int x;
	int count;
	for (int i = 0; i <= NODE_NUM / 32; i++){
		x = p.bitmarker[i];
		for (count = 0; x; count++)
			x &= x - 1;
		sign_num += count;
	}
	return sign_num;
}

//检查最后路径中是否含有路径片段
bool check(ShortPath path_frag, ShortPath long_path){
	ShortPath p;
	for (int i = 0; i <= NODE_NUM / 32; i++){
		p.bitmarker[i] = path_frag.bitmarker[i] & long_path.bitmarker[i];
	}
	if (count_node(p) == count_node(path_frag)){
		return true;
	}
	else
	{
		return false;
	}
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
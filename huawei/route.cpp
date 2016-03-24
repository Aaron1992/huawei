#include "route.h"


const int kMAX_NODES = 600;
const int kMAX_KEEP = 2000;
const int kKEEP = 128;
//int K = 1; //k shortest path

const bool debug = true;
static int v_edge_t[kMAX_NODES][kMAX_NODES] = { -1 };
bool is_loopless(ShortPath s);
int count_in(ShortPath s_path, std::vector<vertex_t> demand);
bool shorter(ShortPath p1, ShortPath p2);

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}


int handle_graph(char **topo, int edge_num, Graph &graph){
	int id, from_id, to_id, cost;
	int node_num, temp;

	node_num = 0;
	for (int i = 0; i < edge_num; i++){
		std::vector<std::string> row = split(topo[i], ',');
		id = std::stoi(row[0]);
		from_id = std::stoi(row[1]);
		to_id = std::stoi(row[2]);
		cost = std::stoi(row[3]);
		graph.add_link(id, from_id, to_id, cost);
		v_edge_t[from_id][to_id] = id;
		//计算节点数
		temp = from_id > to_id ? from_id : to_id;
		if (node_num < temp){
			node_num = temp;
		}
	}
	return node_num +1 ;
}



int read_demand(char *demand, int &start, int &end, std::vector<int> &demand_nodes){
	std::vector<std::string> demand_ = split(demand, ',');
	start = std::stoi(demand_[0]);
	end = std::stoi(demand_[1]);
	std::vector<std::string> s_demand_nodes = split(demand_[2], '|');
	for (auto n : s_demand_nodes){
		demand_nodes.push_back(std::stoi(n));
	}

	return demand_nodes.size();
	//std::cout << "Demand:" << start << "->" << end << "," << demand_nodes << std::endl;
}

void search_route(char *topo[5000], int edge_num, char *demand){
	int keep_max = 1000;
	int last_cost = 100000;
	int count = 0;
	int cost = 0;
	int k = 1;

	int v_start;
	int v_end;
	int node_num;
	int test_count;
	unsigned int demand_vector[19];
	static Graph graph = Graph(kMAX_NODES);

	node_num = handle_graph(topo, edge_num, graph);
	NODE_NUM = node_num;
	graph.adjacency_list.resize(node_num);
	if (node_num < 50) k = 4;
	while (!try_search(graph, node_num, demand, keep_max, k)){
		k++;
	}
	//for (int i = 1; i != 0; i--){
	//	cost = try_search(topo, edge_num, demand, keep_max);
	//	if (cost == last_cost){
	//		count++;
	//	}
	//	else
	//	{
	//		count = 0;
	//	}
	//	if (2 == count)break;
	//	last_cost = cost;
	//	keep_max *= 2;
	//}
}

bool try_search(Graph graph, int node_num, char *demand, int keep_max, int k){
	int v_start;
	int v_end;
	int test_count;
	unsigned int demand_vector[19];
	static std::unordered_map<int, std::unordered_map<int, std::vector<ShortPath>>> m_dis;   //shortest path matirx
	static std::unordered_map<int, std::unordered_map<int, std::vector<std::vector<vertex_t>>>> m_route;
	std::unordered_map<vertex_t, std::vector<ShortPath>> m_f, last_f;

	// read graph
	std::vector<vertex_t> demand_nodes;
	read_demand(demand, v_start, v_end, demand_nodes);
	for (unsigned int &b : demand_vector) b = 0;
	for (int i = 0; i < demand_nodes.size(); i++){
		demand_vector[demand_nodes[i] / 32] += (1 << (demand_nodes[i] % 32));
	}
	if (debug){
		std::cout << "Demand:" << v_start << "->" << v_end << ":";
		for (auto n : demand_nodes){
			std::cout << n << "|";
		}
		std::cout << std::endl;
	}

	NODE_NUM = node_num;
	
	// Matirix to save shortest path between nodes in {start and demand_nodes}
	std::vector<weight_t> min_distance; // source to sink distance 
	std::vector<vertex_t> previous;
	//std::vector<int> test_dij = shortest_path(graph, 14, 197);
	//int test_dij_length = path_length(graph, test_dij);
	//auto test = k_shortest_path(graph, 2, 3, 3);
	//	DijkstraComputePaths(v_start, graph.adjacency_list, min_distance, previous);
	for (auto vi : demand_nodes){
		//std::vector<vertex_t> path = DijkstraGetShortestPathTo(vi, previous);
		std::vector<std::vector<vertex_t>> k_paths = k_shortest_path(graph, v_start, vi, k);
		//ShortPath s_path = ShortPath(min_distance[vi], path);
		for (auto path : k_paths){
			ShortPath s_path = ShortPath(path_length(graph, path), path);
			m_dis[v_start][vi].push_back(s_path);
			m_route[v_start][vi].push_back(path);
		}

	}
	auto test = k_shortest_path(graph, 2, 3, 3);
	test_count = 0;
	for (auto vi : demand_nodes){
		//DijkstraComputePaths(vi, graph.adjacency_list, min_distance, previous);
		for (auto vl : demand_nodes){
			if (vl == vi) continue;// pass self
			std::vector<std::vector<vertex_t>> k_paths = k_shortest_path(graph, vi, vl, k);
			//ShortPath s_path = ShortPath(min_distance[vi], path);
			for (auto path : k_paths){
				ShortPath s_path = ShortPath(path_length(graph, path), path);
				m_dis[vi][vl].push_back(s_path);
				m_route[vi][vl].push_back(path);
			}

		}
	}
	
	// Calculate iterate f
	for (vertex_t vi : demand_nodes){
		//DijkstraComputePaths(vi, graph.adjacency_list, min_distance, previous);
		std::vector<std::vector<vertex_t>> k_paths = k_shortest_path(graph, vi, v_end, k);
		//auto test = k_shortest_path(graph, 2, 3, 3);
		//ShortPath s_path = ShortPath(min_distance[vi], path);
		if (vi == 18){
			;
		}
		for (auto path : k_paths){
			ShortPath s_path = ShortPath(path_length(graph, path), path);
			m_f[vi].push_back(s_path);
			m_route[vi][v_end].push_back(path);
		}
	}

	// Start searching the best path
	ShortPath temp;
	vertex_t vl;
	for (int gen = 1; gen < demand_nodes.size(); gen++){
		keep_max = gen * kKEEP;
		if (keep_max > kMAX_KEEP) keep_max = kMAX_KEEP;
		last_f = m_f;
		m_f.clear();
		if (debug)std::cout << "Gen:" << gen << std::endl;
		for (vertex_t vi : demand_nodes){
			std::priority_queue<ShortPath, std::vector<ShortPath>, PathCompare> fvi_pqueue;
			for (auto &it : last_f){
				vl = it.first;
				if (vi == vl) continue;
				for (int i = it.second.size(); i != 0; i--){
					for (auto path : m_dis[vi][vl]){
						if (confilct(path, it.second[i - 1]))
							continue;
						temp = path + it.second[i - 1];
						if (count(temp, demand_vector) == gen){
							fvi_pqueue.push(temp);
						}
					}

				}
			}
			for (int i = 0; i<keep_max; i++){
				if (fvi_pqueue.empty())break;
				m_f[vi].push_back(fvi_pqueue.top());
				fvi_pqueue.pop();
			}
		}
		for (auto &path_list : m_f){
			if (path_list.second.size() > keep_max)
				path_list.second.resize(keep_max);
		}
		//for (auto test : m_f){
		//	std::cout << test.first << " cost ";
		//	for (int i = 0; i < test.second.size(); i++){
		//		
		//		std::cout << (test.second[i].bitmarker[0] + test.second[i].bitmarker[1]) << "|";
		//	}
		//	std::cout << std::endl;
		//}
	}


	//last step. Join the start point.
	last_f = m_f;
	m_f.clear();
	std::priority_queue<ShortPath, std::vector<ShortPath>, PathCompare> fvi_pqueue;
	for (auto it : last_f){
		vl = it.first;
		for (int i = 0; i < it.second.size(); i++){
			for (auto path : m_dis[v_start][vl]){
				if (confilct(path, it.second[i]))
					continue;
				temp = path + it.second[i];
				if (count(temp, demand_vector) >= demand_nodes.size()){
					fvi_pqueue.push(temp);
				}
			}
		}
	}
	for (int i = 0; i<keep_max; i++){
		if (fvi_pqueue.empty())break;
		m_f[v_start].push_back(fvi_pqueue.top());
		fvi_pqueue.pop();
	}


	std::vector<vertex_t> best_route; //路过的点
	//std::vector<vertex_t> best_tour ; //路过的边
	
	//if (m_f[v_start].size() != 0){
	//	//std::vector<vertex_t> &best_route = m_f[v_start][0].;
	//	m_f[v_start][0].order.push_back(v_end);
	//	auto order = m_f[v_start][0].order;
	//	auto c = count(m_f[v_start][0],demand_vector);
	//	std::cout << c;
	//	auto s = order[0];
	//	for (auto t = order.begin() + 1; t != order.end(); t++){
	//		for (int i = 1;i< m_route[s][*t].size();i++){
	//			best_route.push_back(m_route[s][*t][0][i]);
	//		}
	//		/*int rank = 1;
	//		for (int i = 0; i < demand_nodes.size();i++){
	//			if (m_dis[i][*t].cost < m_dis[s][*t].cost) rank++;
	//		}
	//		std::cout << s << "->" << *t << " rank:" << rank << " cost:" << m_dis[s][*t].cost << std::endl;*/
	//		s = *t;
	//	}
	//	s = v_start;
	//	if (debug){
	//		std::cout << "Cost:" << m_f[v_start][0].cost << std::endl;
	//	}
	//	for (auto t = best_route.begin() ; t != best_route.end(); t++){
	//		best_tour.push_back(v_edge_t[s][*t]);
	//		if (debug){
	//			std::cout << *t << "->";
	//		}
	//		s = *t;
	//	}
	//	for (int i = 0; i < best_tour.size(); i++)
	//		record_result(best_tour[i]);
	//	return m_f[v_start][0].cost;
	//}
	std::vector<vertex_t> best_tour;
	if (m_f[v_start].size() != 0){
		std::vector<vertex_t> &best_route = m_f[v_start][0].path;
		int s = v_start;
		if (debug){
			std::cout << "Cost:" << m_f[v_start][0].cost << std::endl;
		}
		for (auto t = best_route.begin() + 1; t != best_route.end(); t++){
			best_tour.push_back(v_edge_t[s][*t]);
			if (debug){
				std::cout << *t << "->";
			}
			s = *t;
		}
	}
	if (best_tour.size() != 0){
		for (int i = 0; i < best_tour.size(); i++){
			record_result(best_tour[i]);
		}
		return true;
	}
	return false;
}



bool shorter(ShortPath p1, ShortPath p2){
	if (p1.cost < p2.cost){
		return true;
	}
	return false;
}
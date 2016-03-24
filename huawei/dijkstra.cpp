#include "dijkstra.h"

void DijkstraComputePaths(vertex_t source,
	const adjacency_list_t &adjacency_list,
	std::vector<weight_t> &min_distance,
	std::vector<vertex_t> &previous)
{
	int n = adjacency_list.size();
	min_distance.clear();
	min_distance.resize(n, max_weight);
	min_distance[source] = 0;
	previous.clear();
	previous.resize(n, -1);
	// we use greater instead of less to turn max-heap into min-heap
	std::priority_queue<weight_vertex_pair_t,
		std::vector<weight_vertex_pair_t>,
		std::greater<weight_vertex_pair_t> > vertex_queue;
	vertex_queue.push(std::make_pair(min_distance[source], source));

	while (!vertex_queue.empty())
	{
		weight_t dist = vertex_queue.top().first;
		vertex_t u = vertex_queue.top().second;
		vertex_queue.pop();

		// Because we leave old copies of the vertex in the priority queue
		// (with outdated higher distances), we need to ignore it when we come
		// across it again, by checking its distance against the minimum distance
		if (dist > min_distance[u])
			continue;

		// Visit each edge exiting u
		const std::vector<neighbor> &neighbors = adjacency_list[u];
		for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
			neighbor_iter != neighbors.end();
			neighbor_iter++)
		{
			vertex_t v = neighbor_iter->target;
			weight_t weight = neighbor_iter->weight;
			weight_t distance_through_u = dist + weight;
			if (distance_through_u < min_distance[v]) {
				min_distance[v] = distance_through_u;
				previous[v] = u;
				vertex_queue.push(std::make_pair(min_distance[v], v));

			}

		}
	}
}


std::vector<vertex_t> DijkstraGetShortestPathTo(
	vertex_t vertex, const std::vector<vertex_t> &previous)
{
	std::vector<vertex_t> path;
	for (; vertex != -1; vertex = previous[vertex])
		path.push_back(vertex);
	std::reverse(path.begin(), path.end());
	return path;
}

std::vector<vertex_t> shortest_path(Graph &graph, vertex_t source, vertex_t sink){
	std::vector<weight_t> min_distance;
	std::vector<vertex_t> previous;
	DijkstraComputePaths(source, graph.adjacency_list, min_distance, previous);
	std::vector<vertex_t> path = DijkstraGetShortestPathTo(sink, previous);
	if (path.size() == 1) path.clear();
	return path;//return one node means no route
}

int path_length(Graph &graph, std::vector<vertex_t> path){
	int length = 0;
	bool found;
	if (path.size() <= 1) return INF;
	for (int i = 0; i != (path.size() - 1); i++){
		found = false;
		std::vector<neighbor> neighbors = graph.adjacency_list[path[i]];
		for (auto neighbor : neighbors){
			if (neighbor.target == path[i + 1]){
				length += neighbor.weight;
				found = true;
			}
		}
		if (!found)
			length += INF;
	}
	return length;
}
//temp link to save removed link
struct link{
	int id;
	int source;
	int sink;
	int weight;
	link(int arg_id, vertex_t arg_source, vertex_t arg_sink, weight_t arg_weight)
		: id(arg_id), source(arg_source), sink(arg_sink), weight(arg_weight){ }
};

struct remove_node
{
	vertex_t node_id;
	std::vector<neighbor> neighors;
};

struct less_than_path
{
	inline bool operator() (const std::pair<std::vector<vertex_t>, int>& path1, const std::pair<std::vector<vertex_t>, int>& path2)
	{
		return (path1.second < path2.second);
	}
};

std::vector<std::vector<vertex_t>> k_shortest_path(Graph &graph,vertex_t source,vertex_t sink, int k_number){
	// Determine the shortest path from the source to the sink.
	std::vector<std::vector<vertex_t>> result_list;
	result_list.push_back(shortest_path(graph, source, sink));
	if (result_list[0].size() == 0){
		result_list.clear();
			return result_list;//return one node means no route
	}

	// Initialize the heap to store the potential kth shortest path.
	std::vector<std::pair<std::vector<vertex_t>, int>> candidates;
	std::vector<link> removed_links;
	std::vector<remove_node> removed_nodes;

	for (int k = 1; k != k_number; k++){
		// The spur node ranges from the first node to the next to last node in the previous k-shortest path.
		for (int i = 0; i != result_list[k - 1].size() - 2; i++){//TODO: mayby -2?
			removed_links.clear();
			removed_nodes.clear();
			// Spur node is retrieved from the previous k-shortest path, k − 1.
			vertex_t spur_node = result_list[k - 1][i];
			// The sequence of nodes from the source to the spur node of the previous k-shortest path.
			std::vector<vertex_t> root_path(result_list[k - 1].begin(), result_list[k - 1].begin() + i);

			for (auto path : result_list){
				// Remove the links that are part of the previous shortest paths which share the same root path.
				bool equal = true;
				if (path.size() < i){
					equal = false;
				}
				else
				{
					for (int temp = 0; temp < i; temp++){
						if (path[temp] != root_path[temp])
							equal = false;
					}
				}
				if (equal){
					//remove i,i+1;
					for (int temp = 0; temp < graph.adjacency_list[spur_node].size(); temp++){
						//for (auto nei : graph.adjacency_list[i]){
						if (graph.adjacency_list[spur_node][temp].target == path[i + 1]){
							link remove_link(graph.adjacency_list[spur_node][temp].id, \
								spur_node, graph.adjacency_list[spur_node][temp].target,
								graph.adjacency_list[spur_node][temp].weight);
							removed_links.push_back(remove_link);
							graph.adjacency_list[spur_node].erase(graph.adjacency_list[spur_node].begin() + temp);
							break;
						}
					}
				}
			}

			for (auto node : root_path){
				remove_node node_neihbor;
				node_neihbor.node_id = node;
				node_neihbor.neighors = graph.adjacency_list[node];
				removed_nodes.push_back(node_neihbor);
				graph.adjacency_list[node].clear();
			}

			// Calculate the spur path from the spur node to the sink.
			std::vector<vertex_t> spur_path = shortest_path(graph, spur_node, sink);
			if (spur_path.size() == 1)break;//return one node means no route

			// Entire path is made up of the root path and spur path.


			// Add back the edges and nodes that were removed from the graph.
			for (auto link : removed_links){
				graph.add_link(link.id, link.source, link.sink, link.weight);
			}
			for (auto node_neihbor : removed_nodes){
				graph.adjacency_list[node_neihbor.node_id] = node_neihbor.neighors;
			}
			std::vector<vertex_t> total_path;
			total_path.reserve(root_path.size() + spur_path.size());
			total_path.insert(total_path.end(), root_path.begin(), root_path.end());
			total_path.insert(total_path.end(), spur_path.begin(), spur_path.end());
			if (path_length(graph, total_path) < INF && total_path.size() != 1)
			// Add the potential k-shortest path to the heap.
				candidates.push_back(std::make_pair(total_path, path_length(graph, total_path)));

		}
		if (candidates.size() == 0)
			// This handles the case of there being no spur paths, or no spur paths left.
			// This could happen if the spur paths have already been exhausted (added to A), 
			// or there are no spur paths at all - such as when both the source and sink vertices 
			// lie along a "dead end".
			break;
		// Sort the potential k-shortest paths by cost.
		std::sort(candidates.begin(), candidates.end(), less_than_path());
		// Add the lowest cost path becomes the k-shortest path.
		result_list.push_back(candidates[0].first);
		candidates.erase(candidates.begin());
	}

	return result_list;

};
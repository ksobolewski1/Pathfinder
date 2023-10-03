#include "AStar.h"
#include "NavMesh.h"

std::vector<int> A_Star::Find(const NavMesh& mesh) {

	std::vector<int> path; // return vector 
	std::unordered_map<int, Node*> visited; // to keep track of previously visited nodes 
	std::unordered_map<int, Node*> enqueued; // to keep track of enqueued nodes 
	std::vector<Node*> memory_vect; // to keep track of any dynamic allocations

	// accepts a lambda to compare nodes in Heap::HeapUp() and Heap::HeapDown();
	Heap<Node, int> queue = Heap<Node, int>([](Node* n1, Node* n2) {return n1->h_cost + (n1->g_cost * n1->gcost_scalar) < n2->h_cost + (n2->g_cost * n2->gcost_scalar);});

	Node* entry_point = new Node(std::move(mesh.GetEntryPointData()));
	sf::Vector2f destination_pos = mesh.GetNodeData(mesh.GetDestinationID()).position;
	queue.Insert(entry_point);
	memory_vect.push_back(entry_point);

	Node* current = nullptr; 

	auto start = std::chrono::high_resolution_clock::now();

	bool found = false; 
	while (!queue.Empty()) {

		current = queue.RemoveRoot();
		visited.insert({ current->data.ID, current });

		// the algorithm reached its destination 
		if (current->data.ID == mesh.GetDestinationID()) {
			found = true;
			break;
		}

		const std::unordered_map<int, float>& neighbours = current->data.neighbours;

		// iterate through the neighbours, set their costs and enqueue them 
		for (const auto& [id, distance] : neighbours) {

			if (visited.count(id) > 0) continue;

			if (enqueued.count(id) == 0) {

				Node* next = new Node(mesh.GetNodeData(id));
				memory_vect.push_back(next);

				next->parent = current;
				next->SetHCost(next->data.position - destination_pos);
				next->g_cost = current->g_cost + distance;

				queue.Insert(next);
				enqueued.insert({ id, next });
			}
			else {
				// if the neighbour is already enqueued, and the path from current is shorter, then a better path to this neighbour was found 
				if (current->g_cost + distance < enqueued.at(id)->g_cost) {
					Node* n = enqueued.at(id);
					n->g_cost = current->g_cost + distance;
					n->parent = current;
				}
				else continue; 
			}

		}
	}

	// reconstruct the path 
	if (found) {
		Node* current_on_path = current;
		while (current_on_path->parent != nullptr) {
			path.push_back(current_on_path->data.ID);
			current_on_path = current_on_path->parent; 
		}
		path.push_back(current_on_path->data.ID);
		std::reverse(path.begin(), path.end());

		std::cout << "Path found in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " milliseconds\n\n";
	}
	else std::cout << "No valid path found\n";

	for (auto& node : memory_vect) {
		if (node != nullptr) delete node; 
		node = nullptr; 
	}

	return path; 
}

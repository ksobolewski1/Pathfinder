#pragma once

#include "includes.h"

class NavMesh
{
private:

	struct Node {
	private:
		sf::Vector2f position; 
		std::unordered_map<int, float> neighbours;
	public:
		Node(sf::Vector2f pos) : position(pos) {}

		sf::Vector2f GetPosition() const { return position; }
		void SetPosition(sf::Vector2f pos) { position = pos; }

		const std::unordered_map<int, float>& GetNeighbours() const { return neighbours; }
		void AddNeighbour(float dist, int id) { neighbours[id] = dist; }
		void ClearNeighbours() { neighbours.clear(); }
	};

	// to be set by the user 
	int entry_point_id;
	int destination_id;

	std::vector<Node> nodes;
	std::unordered_map<long long int, std::pair<int, int>> edges; 

	// To validate randomly generated nodes 
	bool InsideObstacles(sf::Vector2f pt, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& obstacles);
	bool RectContains(sf::Vector2f pt, const std::pair<sf::Vector2f, sf::Vector2f>& rect_data, float offset);

	// clear edges ahead of Remake() 
	void Clear();

public:

	// a struct for sending Node data to other classes, mainly for A*::Node construction and edge display in Interface 
	struct NodeData {
		const sf::Vector2f position;
		const std::unordered_map<int, float> neighbours;
		const int ID; // for A* to know whether it found the destination 

		NodeData(const Node& node, int id) : position(node.GetPosition()), neighbours(node.GetNeighbours()), ID(id) {}
	};

	NavMesh(int sc_w, int sc_h, const int pt_count, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& obstacles);

	// calls Watson's algorithm - called in NavMesh constructor, and by the Interface in case of node-dragging modifications
	void Remake(int sc_w, int sc_h, const int pt_count, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& obstacles);

	// getters and setters used by Interface 
	std::vector<Node>& GetNodes() { return nodes; }
	const std::unordered_map<long long int, std::pair<int, int>>& GetEdges() { return edges; }

	void SetEntryPoint(int id) { entry_point_id = id; }
	void SetDestination(int id) { destination_id = id; }

	bool StartSelected() { return entry_point_id != -1; }
	bool EndSelected() { return destination_id != -1; }

	// getters used by A* 
	NodeData GetEntryPointData() const { return NodeData(nodes[entry_point_id], entry_point_id); }
	NodeData GetNodeData(int id) const { return NodeData(nodes[id], id); }

	int GetEntryPointID() const { return entry_point_id; }
	int GetDestinationID() const { return destination_id; }

	// Start/end random selection in case the user does not define them 
	void RandomStart();
	void RandomEnd();

};


#pragma once

#include "AStar.h"
#include "NavMesh.h"

class Interface
{

private:

	// cooldown for keyboard presses, in milliseconds 
	int cooldown = 500;
	std::chrono::steady_clock::time_point start;

	// stage of the programme, wraps around from 3->1 
	int stage = 0; 

	// for dragging logic 
	bool dragging = false;
	int drag_node_id = -1; 

	// minimum mesh_size is 3 
	int mesh_size = 3; 

	NavMesh* nav_mesh = nullptr;

	struct Obstacle {

		sf::RectangleShape shape;
		sf::Vector2f dimensions;

		Obstacle(sf::Vector2f pos, sf::Vector2f dim) : shape(dim), dimensions(dim) {
			shape.setOrigin(dimensions.x / 2.0f, dimensions.y / 2.0f); 
			shape.setPosition(pos); 
			shape.setOrigin(0.0f, 0.0f);
			shape.setFillColor(sf::Color::White);
		}
	};

	// Wraps data for drawing NavMesh::Node 
	struct Node {

		sf::CircleShape shape;
		const float radius; 

		Node(sf::Vector2f pos) : radius(5.0f), shape(5.0f) {
			shape.setOrigin(radius, radius);
			shape.setPosition(pos);
		}
		bool Contains(sf::Vector2i pt) const { 
			sf::Vector2f ptf = sf::Vector2f((float)pt.x, (float)pt.y);
			sf::Vector2f diff = ptf - shape.getPosition(); 
			return std::sqrt(diff.x * diff.x + diff.y * diff.y) <= radius; 
		}

	};

	std::vector<Node> nodes;
	std::vector<Obstacle> obstacles;
	std::vector<sf::Vertex> edges; 
	std::unordered_map<int, int> path; 
	
	// Asks the user for the desired mesh size 
	void SetMeshSize();

	// Generates a random obstacle course 
	void GetObstacleDisplay(int sc_w, int sc_h);

	// Get obstacle origin (pair.first) and its width and height (pair.second) for defining valid nodes
	std::vector<std::pair<sf::Vector2f, sf::Vector2f>> GetObstacleData();

	// Get the NavMesh nodes and edges to interact with/draw 
	void GetNodesInterface(); 
	void GetEdgeDisplay();

	// Build the path-map found by the pathfinding algorithm for display in Update()
	void GetPath(std::vector<int> p);

	// Update the nodes interface (selecting start/finish and dragging nodes) and draw, called in Update()
	void UpdateNodes(sf::RenderWindow& win);

	// Clear up every 0/3 stage 
	void Reset();

public:

	Interface() { std::cout << "Press SPACE to generate the obstacles\n\n"; start = std::chrono::high_resolution_clock::now(); }

	// main update function 
	void Update(sf::RenderWindow& win);

	void Test(sf::RenderWindow& win) {
		std::chrono::steady_clock::time_point start = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < 1000; ++i) {
			Reset();
			GetObstacleDisplay(win.getSize().x, win.getSize().y);
			nav_mesh = nav_mesh = new NavMesh(win.getSize().x, win.getSize().y, 1000, GetObstacleData());
			nav_mesh->RandomStart();
			nav_mesh->RandomEnd();
			A_Star::Find(*nav_mesh);
			std::cout << i << "\n";
		}
		Reset();
		std::cout << "Test finished in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " milliseconds\n\n";
	}

};


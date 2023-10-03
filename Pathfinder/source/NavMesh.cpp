#include "NavMesh.h"
#include "Bowyer-Watson.c"

bool NavMesh::InsideObstacles(sf::Vector2f pt, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& obstacles) {
	for (const auto& obs : obstacles) if (RectContains(pt, obs, 5.0f)) return true;
	return false; 
}

bool NavMesh::RectContains(sf::Vector2f pt, const std::pair<sf::Vector2f, sf::Vector2f>& rect_data, float off) {
	return pt.x >= rect_data.first.x - off && pt.x <= rect_data.first.x + rect_data.second.x + off 
		&& pt.y >= rect_data.first.y - off && pt.y <= rect_data.first.y + rect_data.second.y + off;
}

NavMesh::NavMesh(int sc_w, int sc_h, int pt_count, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& obstacles) : entry_point_id(-1), destination_id(-1) {

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> width(0.0f, (float)sc_w);
	std::uniform_real_distribution<float> height(0.0f, (float)sc_h);

	for (int i = 0; i < pt_count; ++i) {

		sf::Vector2f pt = sf::Vector2f(width(gen), height(gen));
		while (InsideObstacles(pt, obstacles)) pt = sf::Vector2f(width(gen), height(gen));
		nodes.emplace_back(Node(pt));
	}

	Remake(sc_w, sc_h, pt_count, obstacles);
}

void NavMesh::Remake(int sc_w, int sc_h, const int pt_count, const std::vector<std::pair<sf::Vector2f, sf::Vector2f>>& obstacles) {

	if (nodes.empty()) return;

	Clear();

	// a circle that encompasses the area on which the mesh is to be defined - used by Bowyer-Watson to define the overarching triangle 
	sf::Vector2f excircle_centre = sf::Vector2f(sc_w / 2.0f, sc_h / 2.0f);
	float excircle_rad = std::sqrt(-excircle_centre.x * -excircle_centre.x + -excircle_centre.y * -excircle_centre.y);

	// convert the mesh data (points and obstacle vertices) into Bowyer-Watson's Point and Rect structs 
	Point* cpoints = new Point[pt_count + 3]; // allocate 3 more points for Bowyer-Watson's super-triangle vertices 
	for (int i = 0; i < pt_count; ++i) {
		struct Point pt;
		pt.x = nodes[i].GetPosition().x;
		pt.y = nodes[i].GetPosition().y;
		pt.id = i;
		cpoints[i] = pt;
	}

	// assemble the obstacles array 
	struct Rect* obs_arr = new Rect[(int)obstacles.size() + 1];
	for (int i = 0; i < obstacles.size(); ++i) {
		Rect obs;
		obs.edges[0] = { {obstacles[i].first.x, obstacles[i].first.y},
			{obstacles[i].first.x, obstacles[i].first.y + obstacles[i].second.y} };
		obs.edges[1] = { {obstacles[i].first.x, obstacles[i].first.y + obstacles[i].second.y},
			{obstacles[i].first.x + obstacles[i].second.x, obstacles[i].first.y + obstacles[i].second.y} };
		obs.edges[2] = { {obstacles[i].first.x + obstacles[i].second.x, obstacles[i].first.y + obstacles[i].second.y},
			{obstacles[i].first.x + obstacles[i].second.x, obstacles[i].first.y} };
		obs.edges[3] = { {obstacles[i].first.x + obstacles[i].second.x, obstacles[i].first.y},
			{obstacles[i].first.x, obstacles[i].first.y } };
		obs_arr[i] = obs; 
	}

	std::cout << "Triangulating the nodes...\n";
	auto start = std::chrono::high_resolution_clock::now();

	// call Bowyer-Watson's triangulation algorithm 
	struct Edge* triangulation = BowyerWatson(pt_count, cpoints, excircle_rad, excircle_centre.x, excircle_centre.y, obs_arr, (int)obstacles.size());

	if (triangulation == nullptr) std::cout << "Triangulation failed\n\n";
	else {
		int index = 0; 
		while (triangulation[index].last != 1) {
			nodes[triangulation[index].start].AddNeighbour(triangulation[index].weight, triangulation[index].end);
			nodes[triangulation[index].end].AddNeighbour(triangulation[index].weight, triangulation[index].start);
			edges[CantorPair(triangulation[index].start, triangulation[index].end)] = std::make_pair(triangulation[index].start, triangulation[index].end);
			++index;
		}
		free(triangulation);
		std::cout << "Triangulation finished in " << 
			std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " milliseconds\n\n";
	}

	if (cpoints != nullptr) delete[] cpoints;
	if (obs_arr != nullptr) delete[] obs_arr;
}



void NavMesh::Clear() {
	edges.clear();
	for (auto& node : nodes) node.ClearNeighbours();
}


void NavMesh::RandomStart() {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> st(0, nodes.size() - 1);
	entry_point_id = st(gen);
}

void NavMesh::RandomEnd() {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> e(0, nodes.size() - 1);
	int end = e(gen);
	while (end == entry_point_id) end = e(gen);
	destination_id = end;
}
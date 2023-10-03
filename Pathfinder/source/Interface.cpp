#include "Interface.h"

void Interface::Update(sf::RenderWindow& win) {

    // proceed to next stage on spacebar press and if cooldown is complete 
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space) && 
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() > cooldown) {

        start = std::chrono::high_resolution_clock::now();

        switch (stage) {

        case 0:
        case 3:
            Reset();
            GetObstacleDisplay(win.getSize().x, win.getSize().y);
            std::cout << "Press SPACE to generate a navigation mesh\n\n";
            break;

        case 1:
            SetMeshSize();
            nav_mesh = new NavMesh(win.getSize().x, win.getSize().y, mesh_size, GetObstacleData()); 
            GetNodesInterface();
            GetEdgeDisplay();

            std::cout << "Left-click on a red node to set the start node\nRight-click on a red node to set the destination\n"; 
            std::cout << "If start/destination is not selected, it will be selected randomly\nPress D and left-click to drag a node and adjust the mesh\n";
            std::cout << "Press SPACE to find the path between selected nodes\n\n";
            break;

        case 2:
            if (!nav_mesh->StartSelected()) nav_mesh->RandomStart();
            if (!nav_mesh->EndSelected()) nav_mesh->RandomEnd();
         
            GetPath(A_Star::Find(*nav_mesh));
            GetEdgeDisplay();

            std::cout << "Press SPACE to re-generate the obstacles\n\n";
            break;
        }
        if (stage == 3) stage = 1;
        else ++stage;
        start = std::chrono::high_resolution_clock::now();
    }

    // dragging detection
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && sf::Keyboard::isKeyPressed(sf::Keyboard::D) && stage == 2) dragging = true;
    else {
        drag_node_id = -1;
        dragging = false;
    }
    
    // drawing 
    for (const Obstacle& obs: obstacles) win.draw(obs.shape);
    win.draw(edges.data(), (int)edges.size(), sf::Lines);
    UpdateNodes(win);
}

void Interface::UpdateNodes(sf::RenderWindow& win) {

    int index = 0; 
    for (Node& node : nodes) {

        // Additional logic bloc for modifying the nodes for the pathfinder 
        if (node.Contains(sf::Mouse::getPosition(win)) && sf::Mouse::isButtonPressed(sf::Mouse::Left)
            && index != nav_mesh->GetDestinationID() && stage == 2 && !dragging) {
            nav_mesh->SetEntryPoint(index);
        }
        else if (node.Contains(sf::Mouse::getPosition(win)) && sf::Mouse::isButtonPressed(sf::Mouse::Right)
            && index != nav_mesh->GetEntryPointID() && stage == 2 && !dragging) {
            nav_mesh->SetDestination(index);
        }

        // dragging a node 
        if (((node.Contains(sf::Mouse::getPosition(win)) && drag_node_id == -1) || drag_node_id == index) && dragging) {
            drag_node_id = index;
            sf::Vector2f mouse_pos = (sf::Vector2f)sf::Mouse::getPosition(win);
            node.shape.setPosition(mouse_pos);
            nav_mesh->GetNodes()[index].SetPosition(mouse_pos);
            nav_mesh->Remake(win.getSize().x, win.getSize().y, mesh_size, GetObstacleData());
            GetEdgeDisplay(); // only edges need to be re-generated 
        }

        // colour entry point and destination 
        if (index == nav_mesh->GetEntryPointID()) node.shape.setFillColor(sf::Color::Green);
        else if (index == nav_mesh->GetDestinationID()) node.shape.setFillColor(sf::Color::Yellow);
        else node.shape.setFillColor(sf::Color::Red);

        // if in the path, change colour to green 
        if (path.count(index) > 0 && index != nav_mesh->GetDestinationID()) node.shape.setFillColor(sf::Color::Green);

        win.draw(node.shape);
        ++index;
    }
}


void Interface::SetMeshSize() {

    std::cout << "Define the mesh size (nr of points): ";

    std::string in;
    while (true) {

        std::getline(std::cin, in);

        if (in.empty()) continue;

        in.erase(std::remove_if(in.begin(), in.end(), [](unsigned char c) { return std::isspace(c); }), in.end());

        try {
            std::stoi(in);
        }
        catch (const std::invalid_argument) {
            std::cout << "Invalid input; expected an integer\nDefine the mesh size (nr of points): ";
            continue;
        }
        int in_ = std::stoi(in);
        if (in_ <= 2) {
            std::cout << "Insufficient mesh size, a minimum of three nodes is required\nDefine the mesh size (nr of points): ";
            continue;
        }

        mesh_size = in_;
        break;
    }
}


void Interface::Reset() {
    nodes.clear();
    obstacles.clear();
    edges.clear();
    path.clear();
    dragging = false; 
    drag_node_id = -1; 
    if (nav_mesh != nullptr) delete nav_mesh;
}


void Interface::GetNodesInterface() {
    nodes.clear();
    for (int i = 0; i < nav_mesh->GetNodes().size(); ++i) nodes.push_back(Node(nav_mesh->GetNodes()[i].GetPosition()));
}


void Interface::GetEdgeDisplay() {
    edges.clear(); 
    for (const auto& [edge_id, pair] : nav_mesh->GetEdges()) {

        sf::Vector2f pos_s = nav_mesh->GetNodeData(pair.first).position;
        sf::Vector2f pos_e = nav_mesh->GetNodeData(pair.second).position;

        sf::Color col = sf::Color::Red;
        if (path.count(pair.first) > 0 && path.count(pair.second) > 0) col = sf::Color::Green;

        edges.push_back(sf::Vertex(pos_s, col));
        edges.push_back(sf::Vertex(pos_e, col));
    }
}


void Interface::GetObstacleDisplay(int sc_w, int sc_h) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> nr(30, 40);
    std::uniform_real_distribution<float> swidth(0.0f, (float)sc_w);
    std::uniform_real_distribution<float> sheight(0.0f, (float)sc_h);
    std::uniform_real_distribution<float> owidth(sc_w * 0.05f, sc_w * 0.1f);
    std::uniform_real_distribution<float> oheight(sc_w * 0.05f, sc_w * 0.1f);

    int obs_nr = nr(gen);

    for (int i = 0; i < obs_nr; ++i) obstacles.push_back(Obstacle(sf::Vector2f(swidth(gen), sheight(gen)), sf::Vector2f(owidth(gen), oheight(gen))));
}


std::vector<std::pair<sf::Vector2f, sf::Vector2f>> Interface::GetObstacleData() {
    std::vector<std::pair<sf::Vector2f, sf::Vector2f>> res;
    for (const auto& obstacle : obstacles)  res.push_back(std::make_pair(obstacle.shape.getPosition(), obstacle.dimensions));
    return res;

}


void Interface::GetPath(std::vector<int> p) { for (int n : p) path[n] = 0; }

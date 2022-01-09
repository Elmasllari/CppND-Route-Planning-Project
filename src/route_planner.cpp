#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;


    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node   = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
//Populating current_node.neighbors vector with all the neighbors.
  current_node->FindNeighbors();

// For each node in current_node.neighbors, set the parent,the h_value,the g_value.
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value +
      current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);
//For each node in current_node.neighbors, add the neighbor to open_list
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

bool compareSumHGvalue(const RouteModel::Node *node_1, const RouteModel::Node *node_2) {
    // NextNode method sort comparator 

    float f1 = node_1->g_value + node_1->h_value;
    float f2 = node_2->g_value + node_2->h_value;

    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
 std::sort(open_list.begin(), open_list.end(), compareSumHGvalue);
    RouteModel::Node* node_lowest_sum = this->open_list.back();
    this->open_list.pop_back();

    return node_lowest_sum;
}

std::vector<RouteModel::Node> 

RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  while (current_node->parent != nullptr) {
    path_found.push_back(*current_node);
    distance += current_node->distance(*current_node->parent);
    current_node = current_node->parent;
  }
    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());
  // Multiplying the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale(); 
    
    return path_found;
}


void RoutePlanner::AStarSearch() {
    // A* Search algorithm
    
    RouteModel::Node *current_node = nullptr;
    
    open_list.emplace_back(start_node);
    start_node->visited = true;

    while (!open_list.empty()) {
        current_node = NextNode();

        if (current_node == end_node) m_Model.path = ConstructFinalPath(current_node);
        
        AddNeighbors(current_node);
    }
}
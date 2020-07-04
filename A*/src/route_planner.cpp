#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    RouteModel::Node* start_node = &(model.FindClosestNode(start_x, start_y));
    RouteModel::Node* end_node = &(model.FindClosestNode(end_x, end_y));

    this->start_node = start_node;
    this->end_node = end_node;
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

}


//Heuristic function to get Manhattan distance to goal

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float distance = node->distance(*(this->end_node));
    return distance;
}


// Expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();
    for(auto neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*(neighbor));
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->visited = true;
        this->open_list.push_back(neighbor);
    }
}


// Sort the open list and return the next node.

bool Compare(RouteModel::Node* n1, RouteModel::Node* n2){
    float f1 = n1->g_value + n1->h_value;
    float f2 = n2->g_value + n2->h_value;

    return (f1>f2);
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), Compare);
    RouteModel::Node* nextnode = this->open_list.back();
    this->open_list.pop_back();
    return nextnode;
}


// Return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);

    while(current_node != this->start_node){
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;  
        path_found.push_back(*current_node);
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    current_node->visited = true;
    while(current_node != nullptr){
        this->AddNeighbors(current_node);
        current_node = this->NextNode();
        if(current_node == this->end_node){
            std::cout << "End Node Reached\n";
            std::vector<RouteModel::Node> path = ConstructFinalPath(current_node);
            this->m_Model.path = path; 
            return;
        }
    }
}
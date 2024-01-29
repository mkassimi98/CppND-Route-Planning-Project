#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(model.FindClosestNode(start_x, start_y)); 
    end_node = &(model.FindClosestNode(end_x,end_y));
        
    if(!start_node)
    {
        start_node = new RouteModel::Node();
    }
    if(!end_node)
    {
        end_node = new RouteModel::Node();
    }
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    if(node && end_node)
    {
        return node->distance(*end_node);
    }
    else{
        return 0;
    }
}

void RoutePlanner::AddNeighbors(RouteModel::Node*current_node) {
    if (!current_node) {
        return;
    }

    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors) {
        if (!neighbor) {
            continue;
        }

        float new_g_value = current_node->g_value + current_node->distance(*neighbor);

        if (!neighbor->visited || new_g_value < neighbor->g_value) {
            neighbor->parent = current_node;
            neighbor->g_value = new_g_value;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->visited = true;
            open_list.push_back(neighbor);
        }
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b)
    {
        float fa = a->g_value + a->h_value;
        float fb = b->g_value + b->h_value;
        
        return fa > fb;    
    });

    RouteModel::Node * neighbor = open_list.back();
    open_list.pop_back();

    return neighbor;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    if(current_node)
    {
        RouteModel::Node *node = current_node;
        while(node->parent != nullptr)
        {
            path_found.push_back(*node);
            distance += node->distance(*(node->parent));
            node = node->parent;
        }
        //start_node has no parent, so It is needed to push it manually to the path_found
        path_found.push_back(*start_node);
        distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters
    }
    
    std::reverse(path_found.begin(), path_found.end());

    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->visited = true;
    open_list.push_back(start_node);

    bool found = false;

    while(!found && !open_list.empty())
    {
        current_node = NextNode();
        if(current_node == end_node)
        {
            found = true;
            m_Model.path = ConstructFinalPath(current_node);
        }
        else
        {
            AddNeighbors(current_node);
        }
    }

}
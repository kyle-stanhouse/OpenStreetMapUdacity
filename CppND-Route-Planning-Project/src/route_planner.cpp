#include "route_planner.h"
#include <algorithm>
#include <iostream>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
  
    //Debug
  	//std::cout << start_node->x << std::endl; 
    //std::cout << start_node->y << std::endl; 

}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  
    return node->distance(*end_node);

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
  	//Call find neighbors on the current node
  	current_node->FindNeighbors();
  
  	//Create a local neighbors vector to loop through
  	// auto neighbors = current_node->neighbors; //could also be done this way with the for loop below
    std::vector<RouteModel::Node *> neighbors_local = current_node->neighbors;
  
    //for (int i = 0; i < neighbors.size(); i++) { //could be done this way but would need to add indexes
     for (auto neighbor : neighbors_local) {
        //Test: EXPECT_PRED2(NodesSame, neighbors[i]->parent, start_node);
      	neighbor->parent = current_node;
        //Test: EXPECT_FLOAT_EQ(neighbors[i]->g_value, start_neighbor_g_vals[i]);
      	neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        //Test: EXPECT_FLOAT_EQ(neighbors[i]->h_value, start_neighbor_h_vals[i]);
        neighbor->h_value = CalculateHValue(neighbor);
      
      	//add to open_list and set visited to true
       	//Test: EXPECT_EQ(neighbors[i]->visited, true);
       	neighbor->visited = true;
      	open_list.push_back(neighbor);

    }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

/*
 
  bool Compare(const vector<int> a, const vector<int> b) {
  int f1 = a[2] + a[3]; // f1 = g1 + h1
  int f2 = b[2] + b[3]; // f2 = g2 + h2
  return f1 > f2; 
  }
  
  void CellSort(vector<vector<int>> *v) {
  sort(v->begin(), v->end(), Compare);
  }
  
*/

//bool Compare(const std::vector<RouteModel::Node*> a, const std::vector<RouteModel::Node*> b){
//bool Compare(const std::vector<RouteModel::Node*> a, const std::vector<RouteModel::Node*> b){
//bool RoutePlanner::Compare(const RouteModel::Node* a, const RouteModel::Node* b){
bool Compare(const RouteModel::Node* a, const RouteModel::Node* b){  

	float cost1 = a->g_value + a->h_value; // cost1 = g1 + h1
  	float cost2 = b->g_value + b->h_value; // cost2 = g2 + h2
 	return cost1 > cost2; 
  
}

RouteModel::Node *RoutePlanner::NextNode() {

  //Sort open list vector of node pointers
  std::sort( open_list.begin(), open_list.end(), Compare );
  //Store lowest sum node
  RouteModel::Node *lowest_sum_node = open_list.back();
  //Remove last Node pointer from open list
  open_list.pop_back();
  
  return lowest_sum_node;
  
  /*
    while (open.size() > 0) {
    // Get the next node
    CellSort(&open);
    auto current = open.back();
    open.pop_back();
    x = current[0];
    y = current[1];
    grid[x][y] = State::kPath;

    // Check if we're done.
    if (x == goal[0] && y == goal[1]) {
      return grid;
    }
    
    // If we're not done, expand search to current node's neighbors.
    // ExpandNeighbors
  }
  */
   
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    
  	// Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  	
  	//Add end node to path
    path_found.push_back(*current_node);

    // TODO: Implement your solution here.
    while(current_node->parent != start_node){
      
      //Update distance traveled 
      distance = distance + current_node->distance(*current_node->parent);
      //Reset current node to the parent, unless we've reached the start node
      current_node = current_node->parent;
      //Add the parent node to the path
      path_found.push_back(*current_node->parent);
      
    }
  	
  	// Add distance to start node
  	distance = distance + current_node->distance(*current_node->parent);
  
  	//Add the start node to the path
    path_found.push_back(*current_node->parent);
  
    //reverse path so that end point is at the end ... 
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  	//RouteModel::Node *current_node = start_node;
  
    //std::cout << current_node->x << std::endl;
  	start_node->parent = nullptr;
    start_node->g_value = 0;
    start_node->h_value = CalculateHValue(start_node);
  	start_node->visited = true;
  
  	// Add start node to open_list vector	
  	open_list.push_back(start_node);
  	//std::cout << start_node->x << std::endl;
    //AddNeighbors(current_node);
  
  	// Execute search
  	//while (current_node->x != end_node->x && current_node->y != end_node->y){
    while( open_list.size() > 0 ){
    //while(current_node != end_node){

      	// Sort open_list and returns lowest sum
        current_node = NextNode();
      
      	// Debug
      	//std::cout << (open_list.size()) << std::endl;
        //std::cout << current_node->x << std::endl;
      
      	if (current_node == end_node){
        //if (current_node->x == end_node->x && current_node->y == end_node->y){
          	m_Model.path = ConstructFinalPath(current_node);
            //std::cout << "found end" << std::endl;
          	//std::cout << current_node->x << std::endl;
            //std::cout << end_node->x << std::endl;
            //std::cout << m_Model.path.back().x << std::endl;
          	//std::cout << start_node->x << std::endl;
            //std::cout << m_Model.path.front().x << std::endl;
            //std::cout << current_node << std::endl;
            return;
        }
          
  		AddNeighbors(current_node);
      
    	//RouteModel::Node *RoutePlanner::NextNode()
      	//Sort open_list and return the lowest cost node, set to the current node 
    	//current_node = NextNode();
  
        //std::cout << current_node->x << std::endl;
	}
  
  	// After the end node is reached, loop back through the parent nodes to save full path
  	//std::cout << "Hi" << std::endl;
  	//m_Model.path = ConstructFinalPath(current_node);

}
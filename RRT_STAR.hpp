#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include <vector>
#include "Node.hpp"
#include <random>
#include "Env.hpp"
#include <limits>
#include <queue>

using namespace std;

//Random initialization
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> dist(0,1);
//Max value for doubles
double infinity = std::numeric_limits<double>::max();

class RRT_STAR{
    public :
    //Constructor
        RRT_STAR(Env environment_arg, Point3 p_start_arg, Point3 p_goal_arg, double step_len_arg, double goal_sample_rate_arg, int iter_max_arg, double search_radius_arg ) : 
        env(environment_arg),
        p_start(p_start_arg),
        p_goal(p_goal_arg),
        step_len(step_len_arg),
        goal_sample_rate(goal_sample_rate_arg), //rate of choosing the node goal as a new random node
        iter_max(iter_max_arg),
        search_radius(search_radius_arg)
        {
            Node * ps_start = new Node(p_start);
            Node * ps_goal = new Node(p_goal);
            this->ps_goal = ps_goal;
            this->ps_start = ps_start;
            vertices.push_back(ps_start); //
        }

    //Destructor
        ~RRT_STAR(){}

//Useful variables
        //Necessary for the constructor
            Point3 p_start;
            Point3 p_goal;
            double step_len;
            double goal_sample_rate;  
            int iter_max;
            Env env;
            double search_radius;

        //Useful information about the scene
            int iter_goal;
            vector<Node*> vertices;   //(Vertex == Sommet en français)
            vector<Node*> tree;
            //Limits of the arena
            double x_min = env.get_x_min();
            double y_min = env.get_y_min();
            double z_min = env.get_z_min();
            double x_max = env.get_x_max();
            double y_max = env.get_y_max();
            double z_max = env.get_z_max();   
            //Rectangles / obstacles
            vector<vector<double>> rectangles = env.get_rectangles();
            //Pointers towards Node start and destination
            Node* ps_start;
            Node* ps_goal;
            //We have taken the same marge for all directions
            double delta_x = env.get_delta_x();
            double delta_y = env.get_delta_y();
            double delta_z = env.get_delta_z();
        //Real limits when margins are accounted
            double x_rand_min = x_min + delta_x;
            double x_rand_max = x_max - delta_x;
            double y_rand_min = y_min + delta_y;
            double y_rand_max = y_max - delta_y;
            double z_rand_min = z_min + delta_z;
            double z_rand_max = z_max - delta_z;

//Useful function 
        //The one necessary for the planning, the afterward functions are mainly used in planning 
        vector<Node*> planning(){
            this->iter_goal = -1;
            int k = 0;
            Node * node_rand;
            Node * node_nearest;
            Node * node_next;
            Node * node_goal;
            Node * node_final_goal = new Node(p_goal);
            while(k < (this->iter_max)){
                node_rand = this->generate_random_node();
                node_nearest = this->vertices[ this->nearest_neighbour(this->vertices,node_rand)];
                node_next = this->new_state(node_nearest, node_rand);
                // delete(node_rand);
                int index = search_goal_parent();
                if(index != (this->vertices.size()-1) && this->iter_goal == -1){
                    this->iter_goal = k;
                }
                if(! (this->env.is_in_collision(node_nearest, node_next)) ){
                    vector<int> neighbour_indexes = find_near_neighbour(node_next);
                    this->vertices.push_back(node_next);
                    if(!neighbour_indexes.empty()){
                        choose_parent(node_next, neighbour_indexes);
                        rewire(node_next, neighbour_indexes);
                    }
                }
                k++;
            }

            for(int i = 1; i < this->vertices.size(); ++i){
                if(this->vertices[i]->parent != nullptr){
                    this->tree.push_back(this->vertices[i]);
                    this->tree.push_back(this->vertices[i]->parent);
                }
            }

            int index = search_goal_parent();
            vector<Node * > path;
            if(index != (this->vertices.size() - 1)){
                return extract_path(this->vertices[index]);
            }
            else{
                std::cout << "No Path found" << std::endl;
                iter_goal = iter_max; 
                vector<Node*> empty_vector;
                return empty_vector;
            }
            //delete(node_final_goal);
        }


        void choose_parent(Node * node_new, vector<int> neighbour_indexes){
            int cost_min_index = 0;
            double cost_temp = infinity;
            for(int k = 0; k < neighbour_indexes.size(); ++k){
                if(cost_temp > get_new_cost(vertices[neighbour_indexes[k]], node_new) ){
                    if(!this->env.is_in_collision( this->vertices[k],node_new)){
                        cost_temp = get_new_cost(vertices[neighbour_indexes[k]], node_new);
                        cost_min_index =neighbour_indexes[k];
                        node_new->set_parent(this->vertices[cost_min_index]);
                    }
                }
            }
            if(!this->env.is_in_collision( this->vertices[cost_min_index],node_new)){
                node_new->set_parent(this->vertices[cost_min_index]);
            }
        }

        void rewire(Node * node_new, vector<int> neighbour_indexes){
            for(int k = 0; k < neighbour_indexes.size(); ++k){
                if(cost(this->vertices[neighbour_indexes[k]]) > get_new_cost(node_new, this->vertices[neighbour_indexes[k]])){
                    if(!this->env.is_in_collision( this->vertices[neighbour_indexes[k]],node_new)){
                        this->vertices[neighbour_indexes[k]]->set_parent(node_new);
                    }
                }
            }
        }

        int search_goal_parent(){
            vector<double> dist_list;
            vector<int> node_index;
            for(int i = 0; i < this->vertices.size(); ++i){
                double dist = distance(vertices[i]->p, p_goal);
                if(dist <= this->step_len){
                    node_index.push_back(i);
                }
                dist_list.push_back(dist);
            }
            
            if(node_index.size() >  0){
                vector<double> cost_list;
                double cost_min = infinity;
                int min_index;
                for(int k = 0; k < node_index.size(); ++k){
                    double temp_cost = dist_list[node_index[k]] + cost(this->vertices[node_index[k]]);
                    bool collision = env.is_in_collision(this->vertices[node_index[k]]->p, p_goal);
                    if(!this->env.is_in_collision(this->vertices[node_index[k]]->p, p_goal)){
                        cost_list.push_back(temp_cost);
                        if(cost_min > temp_cost){
                            cost_min = temp_cost;
                            min_index = node_index[k];
                        }
                    }
                }
                
                if(cost_list.size() > 0){
                     return min_index;
                }
            }
            return this->vertices.size() -1;
        }

        double get_new_cost(Node * node_start, Node * node_end){
            double dist = distance(node_start, node_end);
            return cost(node_start) + dist;
        }

        //Random Node in boundaries
        Node * generate_random_node(){
            if(dist(gen) < goal_sample_rate){
                Node * goal = new Node(p_goal);
                return goal;
            }
            delta_x = env.get_delta_x();
            delta_y = env.get_delta_y();
            delta_z = env.get_delta_z();

            double rand_x_min = x_min+delta_x;
            double rand_x_max = x_max-delta_x;

            double rand_y_min = y_min+delta_y;
            double rand_y_max = y_max-delta_y;

            double rand_z_min = z_min+delta_z;
            double rand_z_max = z_max-delta_z;

            std::uniform_real_distribution<double> dist_x(rand_x_min, rand_x_max);
            std::uniform_real_distribution<double> dist_y(rand_y_min, rand_y_max);
            std::uniform_real_distribution<double> dist_z(rand_z_min, rand_z_max);

            double rand_x = dist_x(gen);
            double rand_y = dist_y(gen);
            double rand_z = dist_z(gen);

            Node* pnode_rand = new Node(Point3(rand_x,rand_y,rand_z)); 
            return pnode_rand;
        }

        vector<int> find_near_neighbour(Node * node_new){
            int n = this->vertices.size() + 1;
            double r;
            if(this->search_radius*(log(n)/n)> this->step_len){
                r = search_radius;
            } 
            else{
                r = this->search_radius*(log(n)/n);
            }
            //La distance de recherche varie (Plus le nombre de sommets visités est faible, plus la distance de recherche sera grande, d'où ln(n)/n)
            vector<double> dist_list;
            for(int i = 0; i < this->vertices.size(); ++i){
                double dist = distance(vertices[i]->p, node_new->p);
                dist_list.push_back(dist);
            }
            vector<int> dist_list_by_index;
            for(int i = 0; i < this->vertices.size(); ++i){
                if( dist_list[i] < r && !this->env.is_in_collision(node_new, this->vertices[i])){
                    dist_list_by_index.push_back(i);
                }
            }
            return dist_list_by_index;
        }

        int nearest_neighbour(vector<Node*>node_list, Node * n){
            int N_list = node_list.size();
            int index  = 0 ;
            double temp = infinity;
            double distance;
            for( int i = 0; i < N_list; ++ i){
                distance = (node_list[i]->p - n->p).norm();
                if(distance < temp){
                    temp = distance;
                    index = i;
                }
            }
            return index;
        }

        //New state based on the chosen random node
        Node * new_state(Node* origin, Node* final){
            Point3 dir = unit_vector(final->p - origin->p);
            double distance = (origin->p - final->p).norm();
            distance = min(this->step_len, distance);
            Node * p_node_new = new Node(Point3(origin->p + distance*dir));
            p_node_new->parent = origin;
            return p_node_new;
        }

        vector<Node*> extract_path(Node * node_end){
            Node * node_start = ps_start;
            vector<Node*> path;
            Node * node_final = new Node(p_goal); 
            path.push_back(node_final);
            ps_goal->set_parent(node_end);
            Node* node_now = node_end;
            int i = 0;
            while(node_now != nullptr){
                path.push_back(node_now);
                node_now = node_now->get_parent();
                ++i;
            }
            //delete(node_final); 
            return path;
        }
            

        double cost(Node * node_p){
            double cost = 0;
            while(node_p->parent != nullptr){
                cost += distance(node_p->parent->p, node_p->p);
                node_p = node_p->parent;
            }
            return cost;
        }

};

inline double get_path_length(vector<Node*> path){
    double length = 0;
    for(int i =0; i < (path.size()-1); ++i){
        length += (path[i+1]->p - path[i]->p).norm();
    }
    return length;
}


#endif

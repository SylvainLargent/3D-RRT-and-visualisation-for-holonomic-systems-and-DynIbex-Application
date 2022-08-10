#ifndef RRT_HPP
#define RRT_HPP

#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include <vector>
#include "Node.hpp"
#include <random>
#include "Env.hpp"
#include <limits>

using namespace std;


std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> dist(0,1);

double infinity = std::numeric_limits<double>::max();

class RRT{
    public :
    //Constructor
        RRT(Env environment_arg, Point3 p_start_arg, Point3 p_goal_arg, double step_len_arg, double goal_sample_rate_arg, int iter_max_arg ) : 
        env(environment_arg),
        p_start(p_start_arg),
        p_goal(p_goal_arg),
        step_len(step_len_arg),
        goal_sample_rate(goal_sample_rate_arg) ,//Probabilité de concevoir un nouveau noeud ? 
        iter_max(iter_max_arg)
        {
            Node * ps_start = new Node(p_start);
            Node * ps_goal = new Node(p_goal);
            this->ps_goal = ps_goal;
            this->ps_start = ps_start;
            vertices.push_back(ps_start); //
        }

    //Destructor
        ~RRT(){}

//Useful variables
        //Necessary argument for the constructor
            Point3 p_start;
            Point3 p_goal;
            double step_len;
            double goal_sample_rate; //Probability of sampling the goal node instead of a new random node
            int iter_max;
            Env env;

        //Useful information about the scene
            int iter_goal;
            vector<Node*> vertices;
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
            //margins in all directions
            double delta_x = env.get_delta_x();
            double delta_y = env.get_delta_y();
            double delta_z = env.get_delta_z();
        //New boundaries limits
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
            int i = 0;
            Node * node_rand;
            Node * node_nearest;
            Node * node_next;
            Node * node_goal;
            Node * node_final_goal = new Node(p_goal);
            while(i < (this->iter_max)){
                node_rand = this->generate_random_node();
                node_nearest = this->vertices[ this->nearest_neighbour(this->vertices,node_rand)];
                node_next = this->new_state(node_nearest, node_rand);
                if(! (this->env.is_in_collision(node_nearest, node_next)) ){
                    this->vertices.push_back(node_next);
                    // this->tree.push_back(node_nearest);
                    // this->tree.push_back(node_next);
                    delete(node_rand);
                    double distance = ((node_final_goal->p) - (node_next->p)).norm();
                    if(distance <= (this->step_len) && (this->iter_goal)==-1){
                        this->iter_goal = i+1;
                        node_goal = node_next;
                    }
                }
                i++;
            }

            for(int i = 1; i < this->vertices.size(); ++i){
                if(this->vertices[i]->parent != nullptr && this->vertices[i] != nullptr){
                    this->tree.push_back(this->vertices[i]);
                    this->tree.push_back(this->vertices[i]->parent);
                }
            }

            if(iter_goal == -1){
                    std::cout << "No Path Found" << std::endl;
                    iter_goal = iter_max; 
                    vector<Node*> empty_vector;
                    return empty_vector;
                }
            else{
                vector<Node*> path1 = extract_path(node_goal);
                return path1;
            }
            delete(node_final_goal);
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
            delete(node_final); 
            return path;
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
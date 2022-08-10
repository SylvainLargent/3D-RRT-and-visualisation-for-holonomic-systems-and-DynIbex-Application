#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Env.hpp"
#include "RRT.hpp"
#include "Node.hpp"
#include <fstream>



int main(int argc, char ** argv){
    // Env environment = Env(-2, 2, -2.5, 2.5, 0, 3, 0.25, 0.25, 0.099);
    // (x_min, x_max, y_min, y_max, z_min, z_max, delta_x, delta_y, delta_z)
    // deltas represents the margin we allowed between the trajectory and the obstacles

    //Voliere de l'U2IS
    Env environment = Env(-1.5, 1.4, -1.4, 1.8, -0.1, 2, 0.25, 0.25, 0.099);

//Rectangular obstacles (Coordinates of the lower corner close to the origin, then thickness in each direction)...
    // vector<vector<double>> list_rectangles =
    //     {
    //         {-3, 3, 0, 2.70, 2, 3},
    //         {0.7, 3, 0, 2.3, 2, 3}
    //     };
//Obstacles
    // vector<vector<double>> list_rectangles =
    //     {
    //         {-3, 4, 0, 6, 1, 2},
    //        {-3, 1, 1, 6, 1, 3}
    //     };

//Amusons nous à faire un labyrinthe
    vector<vector<double>> list_rectangles =
    {
        //    {-3, 0, 0, 4, 1, 3},
        //    {-2, 3, 0, 5, 1, 3},
        //    {-3, 5, 0, 2.5, 1, 3},
        //    {0.5, 5, 0, 2.5, 1, 3}
    };

    //Starting point and goal destination
    Point3 s_goal  =  Point3(-0.917, -0.92, 1);
    Point3 s_start  =  Point3(-0.8, 0.80, 1);

    //Number of iterations and step length at each step
    int iter_max = atoi(argv[1]);
    double step_len = atof(argv[2]);
    double goal_sample_rate = 0.1;
    
//Adding rectangles to the scene
    for(int i = 0; i < list_rectangles.size(); ++i){
        environment.add_rectangle(list_rectangles[i]);
    }

RRT rrt_algo = RRT(environment, s_start, s_goal, step_len, goal_sample_rate,iter_max);


//Giving out, arena or scene necessary info !
    ofstream arena_boundaries;
    arena_boundaries.open("csv_files/boundaries.txt");
    arena_boundaries << environment.get_x_min() << std::endl;
    arena_boundaries << environment.get_x_max() << std::endl;
    arena_boundaries << environment.get_y_min() << std::endl;
    arena_boundaries << environment.get_y_max() << std::endl;
    arena_boundaries << environment.get_z_min() << std::endl;
    arena_boundaries << environment.get_z_max() << std::endl;
    arena_boundaries.close();

    ofstream start_destination;
    start_destination.open("csv_files/start_destination.txt");
    start_destination << (*rrt_algo.ps_start) << std::endl;
    start_destination << (*rrt_algo.ps_goal) << std::endl;   
    start_destination.close();  
    


//Plotting rectangle obstacles thanks to their vertices
    vector<vector<Node*>> all_boundaries = rrt_algo.env.get_obstacles_vertex();  
    ofstream all_boundaries_stream;
    all_boundaries_stream.open("csv_files/obstacles.txt");
    if(!all_boundaries.empty()){
        for(int i = 0; i < all_boundaries.size(); ++i){
            for(int j = 0; j < all_boundaries[i].size();++j){
                all_boundaries_stream << (*(all_boundaries[i][j])) << std::endl;
                delete(all_boundaries[i][j]);
            }
        }
    }
    all_boundaries_stream.close();

//Plotting path trajectory
    vector<Node*> path = rrt_algo.planning();  
    ofstream path_stream;
    path_stream.open("csv_files/trajectory.txt");
    if(!path.empty()){
        for(int i = 0; i < path.size(); ++i){
            path_stream << (*path[i]) << std::endl;
        }
        path_stream << rrt_algo.iter_goal << std::endl;  
    }
    path_stream.close();
    if(!path.empty()){
        std::cout << "Path length : " << get_path_length(path) << std::endl;
    }


//Plotting tree 
    vector<Node*> tree = rrt_algo.tree;  
    ofstream tree_stream;
    tree_stream.open("csv_files/tree.txt");
    if(!tree.empty()){
        for(int i = 0; i < tree.size(); ++i){
            tree_stream << (*tree[i]) << std::endl;
        }
    }
    tree_stream.close();


    return 0;
}

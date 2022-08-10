#ifndef ENV_HPP
#define ENV_HPP

#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Node.hpp"
#include <vector>

using namespace std;

class Env{
    private :
    //Boundaries
        double x_min;
        double x_max;
        double y_min;
        double y_max;
        double z_min;
        double z_max;

    //Number of rectangles and 6 being the parameters to define our rectangles
        int number_of_rectangles = 0;
        // (x_min, x_max, y_min, y_max, z_min, z_max, delta_x, delta_y, delta_z)
        // deltas represents the margin we allowed between the trajectory and the obstacles
        // Indeed to define a new rectangle you'd need 6 arguments

    //Scene 
        vector<vector<double>> rectangles;

        //To define a frame as an obstacle
        //Let's define it with 4 rectangles stuck together
        //(First let's use a simple rectangle)
        //in{{4][6] frame = ..}}; 

        //Delta marge que l'on s'autorise autour des obstaclesc (par exemple la taile du drone)
        double delta_x;
        double delta_y;
        double delta_z;

    public :
    //Constructors 
        Env(){}
        Env(double x_min_arg, double x_max_arg, double y_min_arg, double y_max_arg, double z_min_arg, double z_max_arg, double delta_arg_x,double delta_arg_y,double delta_arg_z): x_min(x_min_arg), x_max(x_max_arg), 
                    y_min(y_min_arg), y_max(y_max_arg), z_min(z_min_arg), z_max(z_max_arg), delta_x(delta_arg_x), delta_y(delta_arg_y), delta_z(delta_arg_z){}

    //Getters
        double get_x_min(){
            return x_min;
        }
        double get_y_min(){
            return y_min;
        }
        double get_z_min(){
            return z_min;
        }
        double get_x_max(){
            return x_max;
        }
        double get_y_max(){
            return y_max;
        }
        double get_z_max(){
            return z_max;
        }

        double get_delta_x(){
            return delta_x;
        }
        double get_delta_y(){
            return delta_y;
        }
        double get_delta_z(){
            return delta_z;
        }

        vector<vector<double>> get_rectangles(){
            return rectangles;
        }
        //Numérotation des différents coins
        //          6***********************7
        //         **                      **
        //        * *                     * *
        //       *  *                    *  *
        //      2**********************3    *   epaisseur z
        //      *   *                  *    *                         
        //      *   *                  *    *                         
        //      *   *                  *    *                         
        //      *   5***********************8                         
        //      *  *                   *   *
        //      * *                    *  * epaisseur y
        //      *                      * *                            
        //      1**********************4         
        //             epaisseur x                  

        //List of the obstacles' vertices (All 8 of them) (and margins included)
        vector<vector<vector<double>>> get_obs_vertex(){ 
            vector<vector<vector<double>>> obs_list;
            int i = 0;
            double ox, oy, oz, ex, ey, ez;
            while( i < number_of_rectangles){
                ox = rectangles[i][0];
                oy = rectangles[i][1];
                oz = rectangles[i][2];
                ex = rectangles[i][3];
                ey = rectangles[i][4];
                ez = rectangles[i][5];
                obs_list.push_back({
                    {ox - delta_x, oy - delta_y, oz - delta_z},
                    {ox - delta_x, oy - delta_y, oz + ez + delta_z},
                    {ox + ex + delta_x, oy - delta_y, oz + ez + delta_z},
                    {ox + ex + delta_x, oy - delta_y, oz - delta_z},
                    {ox - delta_x, oy + ey + delta_y, oz - delta_z},
                    {ox - delta_x, oy + ey + delta_y, oz + ez + delta_z},
                    {ox + ex + delta_x, oy + ey + delta_y, oz + ez + delta_z},
                    {ox + ex + delta_x, oy + ey + delta_y, oz - delta_z}                    
                });  // vertex 1,2 ... till 8
                ++i;
            }
            return obs_list;
        }

        vector<vector<double>> get_rectangles_boundaries(){    //Same but of the boundaries this time !
            vector<vector<double>> boundaries_list;
            int i = 0;
            double ox, oy, oz, ex, ey, ez;
            while( i < number_of_rectangles){
                ox = rectangles[i][0];
                oy = rectangles[i][1];
                oz = rectangles[i][2];
                ex = rectangles[i][3];
                ey = rectangles[i][4];
                ez = rectangles[i][5];
                boundaries_list.push_back({
                    ox,
                    ox +ex,
                    oy,
                    oy+ey,
                    oz,
                    oz+ez,
                });  
                ++i;
            }
            return boundaries_list;
        }

        vector<vector<double>> get_rectangles_inflated_boundaries(){    //Same but of the boundaries this time !
            vector<vector<double>> boundaries_list;
            int i = 0;
            double ox, oy, oz, ex, ey, ez;
            while( i < number_of_rectangles){
                ox = rectangles[i][0];
                oy = rectangles[i][1];
                oz = rectangles[i][2];
                ex = rectangles[i][3];
                ey = rectangles[i][4];
                ez = rectangles[i][5];
                boundaries_list.push_back({
                    ox-delta_x,
                    ox+delta_x +ex,
                    oy-delta_y,
                    oy+delta_y+ey,
                    oz-delta_z,
                    oz+delta_z+ez,
                });  
                ++i;
            }
            return boundaries_list;
        }

        void update_obs(int number_of_rectangles, vector<vector<double>> new_rectangles){
            this->number_of_rectangles = number_of_rectangles;
            this->rectangles = new_rectangles;
        }

        void add_rectangle(vector<double> rectangle){
            this->number_of_rectangles += 1;
            this->rectangles.push_back(rectangle);
        }

        //is the trajectory inside the boundaries ?
        bool trajectory_in_boundaries(Point3 start, Point3 end){
            if(start.x() < x_min || start.x() > x_max || start.y() < y_min || start.y() > y_max){
                return false;
            }
            if(end.x() < x_min || end.x() > x_max || end.y() < y_min || end.y() > y_max){
                return false;
            }
            return true;
        }

        //Overloading the function
        bool collide_with_a_rectangle(Segment u){
            Point3 start = u.get_start();
            Point3 end   = u.get_end();
            double x_min, x_max, y_min, y_max, z_min, z_max;
            vector<vector<double>> boundaries_list = this->get_rectangles_inflated_boundaries();
            int i = 0;
            double step = 0.0001;
            double distance;
            Point3 dir = unit_vector(end - start); //Here represents a vector
            double dist_start_end = (end - start).norm();
            Point3 ray;
            while(i<number_of_rectangles){
                distance = 0;
                x_min = boundaries_list[i][0];
                x_max = boundaries_list[i][1];
                y_min = boundaries_list[i][2];
                y_max = boundaries_list[i][3];
                z_min = boundaries_list[i][4];
                z_max = boundaries_list[i][5];
                while(distance < dist_start_end){
                    ray = start +  (distance*dir);
                    if( x_min < ray.x() && ray.x() < x_max && y_min < ray.y() && ray.y() < y_max && z_min < ray.z() && ray.z() < z_max){
                        return true;
                    }
                    distance += step;
                }
                ++i;
            }
            return false;
        }

        bool collide_with_a_rectangle(Point3 start, Point3 end){
            double x_min, x_max, y_min, y_max, z_min, z_max;
            vector<vector<double>> boundaries_list = this->get_rectangles_inflated_boundaries();
            int i = 0;
            double step = 0.1;
            double distance;
            double dist_start_end = (end - start).norm();
            Point3 dir = unit_vector(end - start); //Here represents a vector
            Point3 ray;
            while(i<number_of_rectangles){
                distance = 0;
                x_min = boundaries_list[i][0];
                x_max = boundaries_list[i][1];
                y_min = boundaries_list[i][2];
                y_max = boundaries_list[i][3];
                z_min = boundaries_list[i][4];
                z_max = boundaries_list[i][5];
                while(distance < dist_start_end){
                    ray = start +  (distance*dir);
                    if( x_min < ray.x() && ray.x() < x_max && y_min < ray.y() && ray.y() < y_max && z_min < ray.z() && ray.z() < z_max){
                        return true;
                    }
                    distance += step;
                }
                ++i;
            }
            return false;
        }

        bool is_in_collision(Segment u){
            Point3 start = u.get_start();
            Point3 end   = u.get_end();
            //Is the trajectory in the boundaries ?
            if(start.x() < (x_min + delta_x) || start.x() > (x_max - delta_x) || start.y() < (y_min + delta_y) || start.y() > (y_max - delta_y) || start.z() < (z_min + delta_z) || start.z() > (z_max - delta_z)){
                return true;
            }
            if(end.x() < (x_min + delta_x) || end.x() > (x_max - delta_x) || end.y() < (y_min + delta_y) || end.y() > (y_max - delta_y) || end.z() < (z_min + delta_z) || end.z() > (z_max - delta_z)){
                return true;
            }
            bool last_bool = collide_with_a_rectangle(u);
            return last_bool;
        }

        bool is_in_collision(Point3 start, Point3 end){
            //Is the trajectory in the boundaries ?
             if(start.x() < (x_min + delta_x) || start.x() > (x_max - delta_x) || start.y() < (y_min + delta_y) || start.y() > (y_max - delta_y) || start.z() < (z_min + delta_z) || start.z() > (z_max - delta_z)){
                return true;
            }
            if(end.x() < (x_min + delta_x) || end.x() > (x_max - delta_x) || end.y() < (y_min + delta_y) || end.y() > (y_max - delta_y) || end.z() < (z_min + delta_z) || end.z() > (z_max - delta_z)){
                return true;
            }
            bool last_bool = collide_with_a_rectangle(start,end);
            return last_bool;
        }

        bool is_in_collision(Node * start_n, Node * end_n){
            Point3 start = (*start_n).p;
            Point3 end = (*end_n).p;
            //Is the trajectory in the boundaries ?
             if(start.x() < (x_min + delta_x) || start.x() > (x_max - delta_x) || start.y() < (y_min + delta_y) || start.y() > (y_max - delta_y) || start.z() < (z_min + delta_z) || start.z() > (z_max - delta_z)){
                return true;
            }
            if(end.x() < (x_min + delta_x) || end.x() > (x_max - delta_x) || end.y() < (y_min + delta_y) || end.y() > (y_max - delta_y) || end.z() < (z_min + delta_z) || end.z() > (z_max - delta_z)){
                return true;
            }
            bool last_bool = collide_with_a_rectangle(start,end);
            return last_bool;
        }

        vector<vector<Node*>> get_obstacles_vertex(){    //Get all 8 vertices of each rectangle
            vector<vector<Node*>> all_boundaries_list;
            int i = 0;
            Point3 vertex;
            double ox, oy, oz, ex, ey, ez;
            while( i < number_of_rectangles){
                vector<Node*> boundaries_list;
                
                ox = rectangles[i][0];
                oy = rectangles[i][1];
                oz = rectangles[i][2];
                ex = rectangles[i][3];
                ey = rectangles[i][4];
                ez = rectangles[i][5];
               
                boundaries_list.push_back(new Node(Point3(ox,oy,oz)));
                boundaries_list.push_back(new Node(Point3(ox+ex,oy,oz)));
                boundaries_list.push_back(new Node(Point3(ox+ex,oy+ey,oz)));
                boundaries_list.push_back(new Node(Point3(ox,oy+ey,oz)));
                boundaries_list.push_back(new Node(Point3(ox,oy,oz+ez)));
                boundaries_list.push_back(new Node(Point3(ox+ex,oy,oz+ez)));
                boundaries_list.push_back(new Node(Point3(ox+ex,oy+ey,oz+ez)));
                boundaries_list.push_back(new Node(Point3(ox,oy+ey,oz+ez)));

                //Numérotation des différents coins
                //          7***********************6
                //         **                      **
                //        * *                     * *
                //       *  *                    *  *
                //      4**********************5    *   epaisseur z
                //      *   *                  *    *                         
                //      *   *                  *    *                         
                //      *   *                  *    *                         
                //      *   3***********************2                         
                //      *  *                   *   *
                //      * *                    *  * epaisseur y
                //      *                      * *                            
                //      0**********************1         
                //             epaisseur x                  

                all_boundaries_list.push_back(boundaries_list);

                ++i;
            }
            return all_boundaries_list;
        }

        vector<vector<Node*>> get_inflated_obs_vertex(){  //Inflated obs_vertex
            vector<vector<Node*>> all_boundaries_list;
            int i = 0;
            double ox, oy, oz, ex, ey, ez;
            while( i < number_of_rectangles){
                vector<Node*> boundaries_list;
                ox = rectangles[i][0];
                oy = rectangles[i][1];
                oz = rectangles[i][2];
                ex = rectangles[i][3];
                ey = rectangles[i][4];
                ez = rectangles[i][5];
                boundaries_list.push_back(new Node(Point3(ox - delta_x, oy - delta_y, oz - delta_z)));
                boundaries_list.push_back(new Node(Point3(ox - delta_x, oy - delta_y, oz + ez + delta_z)));
                boundaries_list.push_back(new Node(Point3(ox + ex + delta_x, oy - delta_y, oz + ez + delta_z)));
                boundaries_list.push_back(new Node(Point3(ox + ex + delta_x, oy - delta_y, oz - delta_z)));
                boundaries_list.push_back(new Node(Point3(ox - delta_x, oy + ey + delta_y, oz - delta_z)));
                boundaries_list.push_back(new Node(Point3(ox - delta_x, oy + ey + delta_y, oz + ez + delta_z)));
                boundaries_list.push_back(new Node(Point3(ox + ex + delta_x, oy + ey + delta_y, oz + ez + delta_z)));
                boundaries_list.push_back(new Node(Point3(ox + ex + delta_x, oy + ey + delta_y, oz - delta_z)));                   
                all_boundaries_list.push_back(boundaries_list);  // vertex 1,2 ... till 8
                ++i;
            }
            return all_boundaries_list;
        }

};

#endif
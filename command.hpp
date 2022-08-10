#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Env.hpp"
#include "RRT.hpp"
#include "Node.hpp"
#include <fstream>

vector<vector<double>> determine_cmd(vector<Node*> path, double speed_norm){
    double dist;
    Node* parent;
    Node* child;
    Point3 dir;
    double vx, vy, vz;
    vector<vector<double>> list_of_velocities;
    for(int i = path.size()-2; i > -1; --i){
            child = path[i];
            parent = path[i+1];
            dist = distance(parent,child);
            dir  = unit_vector((child->p-parent->p));
            vx = dir[0]*speed_norm;
            vy = dir[1]*speed_norm;
            vz = dir[2]*speed_norm;
            vector<double> velocity;
            velocity.push_back(vx);
            velocity.push_back(vy);
            velocity.push_back(vz);
            list_of_velocities.push_back(velocity);
    }
    return list_of_velocities;
}

vector<double> determine_time(vector<Node*> path, double speed_norm){
    double dist;
    Node* parent;
    Node* child;
    Point3 dir;
    double time;
    vector<double> pduration_list;
    for(int i = 1; i < path.size(); ++i){
            child = path[i];
            parent = path[i-1];
            dist = distance(parent,child);
            dir  = unit_vector((child->p-parent->p));
            time = dist/speed_norm;
            pduration_list.push_back(time);
    }
    return pduration_list;
}





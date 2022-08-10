#ifndef NODE_HPP
#define NODE_HPP

#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include <vector>

using namespace std;

class Node{
    public :
        //Constructor
        Node() {parent = nullptr;}
        Node(Point3 p_arg): x(p_arg.x()), y(p_arg.y()), z(p_arg.z()), p(p_arg) {
            parent = nullptr;
        }
        //Coordinates of the point
        double x,y,z;
        //Point3 withholding the information x,y,z
        Point3 p;

        void set_parent(Node* parent_arg){
            this->parent = parent_arg;
        }

        Node* get_parent(){
            return this->parent;
        }

        Node *parent = nullptr;      
};

inline std::ostream& operator<<(std::ostream &out, const Node& v) {
    return out << v.p.e[0] << "," << v.p.e[1] << "," << v.p.e[2];
}

inline double distance(Node* u, Node* v){
    return (u->p - v->p).norm();
}


#endif
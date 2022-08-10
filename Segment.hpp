#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include <iostream>
#include <cmath>
#include "Point3.hpp"

class Segment{
    public :
    //Constructeurs
        Segment(Point3 start, Point3 end) : starting_point(start), ending_point(end) {}

    //Getter
        Point3 get_start(){
            return this->starting_point;
        }
        
        Point3 get_end(){
            return this->ending_point;
        }

    private :
        Point3 starting_point;
        Point3 ending_point;
};

#endif
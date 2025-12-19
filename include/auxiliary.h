#pragma once
#include "baseinclude.h"
#include "Point.h"

namespace auxiliary {
    namespace point {
        bool closest_point_on_line(Point &prod, const Point &p1, const Point &p2, const Point &p, char type = 'S') {
            double prj = (p2 - p1)^(p-p1);
            if (prj < 0 && type != 'L') {
                prod = p1;
                return true;
            } {}
            double len = (p2 - p1).mag();
            if (prj > len && type == 'S') {
                prod = p2;
                return true;
            }
            prod = (p1-p2) / len * prj + p1;
            return true;
        }
        // bool get_line_intersection(Point &prod, const Point &p1, const Point &p2, const Point &p3, const Point &p4) {
        //     Point d = p2 - p1;;
        //     double prj1 = 
        // }
    }
    namespace number {

    }
}
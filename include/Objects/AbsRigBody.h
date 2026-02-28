#include <Circle.h>

class AbsRigBody : public Circle {
    private:
    Point vel;
    Point acc;
    public:
    AbsRigBody(Point pos, double r, Point v, Point a) {
        Circle(pos, r);
        vel = v;
        acc = a;
    }
}
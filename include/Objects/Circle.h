#include <Object.h>

class Circle : public Object {
    protected:
    double rad;
    public:
    Circle(Point pos, double r) {
        Ball(pos);
        rad = r;
    }
}
#include "Includes.h"
class Axis
{
private:
    GenericHID thisController;
    int axisNumber;
    bool invert;
public:
    Axis(int controllerNum,int axisNumber, bool invert = false);
    double position(); // returns a value between -1 and 1
};

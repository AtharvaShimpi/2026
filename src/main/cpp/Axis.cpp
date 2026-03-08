#include "Axis.h"

Axis::Axis(int controllerNum, int axisNumber, bool invert)
    : thisController(controllerNum), axisNumber(axisNumber), invert(invert) // Initialize thisController using initializer list
{
}
double Axis::position()
{
    return thisController.GetRawAxis(axisNumber) * (invert ? -1 : 1);
}
// returns a value between -1 and 1
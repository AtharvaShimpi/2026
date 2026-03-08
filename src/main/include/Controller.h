#include "Includes.h"
#include "Button.h"
#include "Axis.h"
class Controller
{
private:
    /* data */
    int thisControllerNum;
public:
    Controller(int controllerNum);
    
    Button ButtonB;
    Button ButtonA;
    Button ButtonY;
    Button ButtonX;
    Button ButtonL1;
    Button ButtonR1;
    Button ButtonL2;
    Button ButtonR2;
    Button ButtonO;
    Button ButtonUp;
    Button ButtonDown;
    Button ButtonLeft;
    Button ButtonRight;
    Axis Axis1;
    Axis Axis2;
    Axis Axis3;
    Axis Axis4;

    void update();
};
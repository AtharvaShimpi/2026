#include "Controller.h"

Controller::Controller(int controllerNum)
    : ButtonB(controllerNum, 1),
      ButtonA(controllerNum, 2),
      ButtonY(controllerNum, 3),
      ButtonX(controllerNum, 4),
      ButtonL1(controllerNum, 5),
      ButtonR1(controllerNum, 6),
      ButtonL2(controllerNum, 7),
      ButtonR2(controllerNum, 8),
      ButtonO(controllerNum, 9),
      ButtonUp(controllerNum, 13),
      ButtonDown(controllerNum, 14),
      ButtonLeft(controllerNum, 15),
      ButtonRight(controllerNum, 16),
      Axis1(controllerNum, 2),
      Axis2(controllerNum, 3, true),
      Axis3(controllerNum, 1, true),
      Axis4(controllerNum, 0)
{
}

void Controller::update()
{
    ButtonB.update();
    ButtonA.update();
    ButtonY.update();
    ButtonX.update();
    ButtonL1.update();
    ButtonR1.update();
    ButtonL2.update();
    ButtonR2.update();
    ButtonO.update();
    ButtonUp.update();
    ButtonDown.update();
    ButtonLeft.update();
    ButtonRight.update();
}
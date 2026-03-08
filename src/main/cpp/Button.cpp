#include "Button.h"

Button::Button(int controllerNum, int buttonNumber)
    : thisController(controllerNum) // Initialize thisController using initializer list
{
    thisButtonNumber = buttonNumber;
    Button::update();
}

void Button::update()
{
    pastState = currentState;
    currentState = thisController.GetRawButton(thisButtonNumber);
}

void Button::changeButton(int buttonNumber)
{
    thisButtonNumber = buttonNumber;
}

bool Button::isPressing()
{
    return currentState;
}

bool Button::isReleased()
{
    return !currentState;
}

bool Button::hasBeenReleased()
{
    return pastState && !currentState;
}

bool Button::hasBeenBumped()
{
    return !pastState && currentState;
}
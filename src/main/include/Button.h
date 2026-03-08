#include "Includes.h"
class Button
{
    private:
    GenericHID thisController;
    bool pastState;
    bool currentState;
    int thisButtonNumber;
    public:
    Button(int controllerNum, int buttonNumber);
    void changeButton(int buttonNumber);
    bool isPressing();
    bool isReleased();
    bool hasBeenReleased();
    bool hasBeenBumped();
    void update();
};
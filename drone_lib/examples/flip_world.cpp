
#include <iostream>

#include <ctello.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

int main()
{
    ctello::Tello tello;
    if (!tello.Bind())
    {
        return 0;
    }

    tello.SendCommand("takeoff");
    // Wait for response
    while (!(tello.ReceiveResponse()));

    tello.SendCommand("flip b");
    while (!(tello.ReceiveResponse()));

    tello.SendCommand("land");
    while (!(tello.ReceiveResponse()));

    return 0;
}
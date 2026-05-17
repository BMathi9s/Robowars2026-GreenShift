#pragma once
#include <Arduino.h>

class Motors {
public:
    static void init();
    
    /**
     * @param left  Power from -255 to 255
     * @param right Power from -255 to 255
     */
    static void setSpeeds(int left, int right);
};
package com.armabot.lidar.impl.vl6180x;

public class Calculations {
    public static short constrain(short value, short min, short max) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        }
        return value;
    }
}

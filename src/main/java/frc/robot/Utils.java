// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

/** A set of utility functions*/
public class Utils {
    public static double smoothStep(double edge0, double edge1, double x, double start, double end) {
        if(edge0 == edge1)
            return start;
        // Scale, and clamp x to 0..1 range
        x = MathUtil.clamp((x - edge0) / (edge1 - edge0), 0, 1);

        return (end - start) * (x * x * (3.0f - 2.0f * x)) + start;
    }

    public static double smootherStep(double edge0, double edge1, double x, double start, double end) {
        if(edge0 == edge1)
            return start;
        // Scale, and clamp x to 0..1 range
        x = MathUtil.clamp((x - edge0) / (edge1 - edge0), 0, 1);

        return (end - start) * (x * x * x * (x * (6.0f * x - 15.0f) + 10.0f)) + start;
    }

    /**Returns a factor which starts at 1, and decreases to approach 0 as current passes from approachStart to target*/
    public static double approachFactor(double current, double target, double approachStart){
        //if in order, current, approachStart, target, either ascending or descending, then we are not in approach
        double approachRange = target - approachStart;
        double dist = target - current;
        if(Math.abs(dist) > Math.abs(approachRange)) //not in approach range yet
            return 1; //continue as we are
        return dist / approachRange;
    }
}
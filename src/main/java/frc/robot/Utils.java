// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** A set of utility functions */
public class Utils {
    public static double smoothStep(double edge0, double edge1, double x, double start, double end) {
        if (edge0 == edge1)
            return start;
        // Scale, and clamp x to 0..1 range
        x = MathUtil.clamp((x - edge0) / (edge1 - edge0), 0, 1);

        return (end - start) * (x * x * (3.0f - 2.0f * x)) + start;
    }

    public static double smootherStep(double edge0, double edge1, double x, double start, double end) {
        if (edge0 == edge1)
            return start;
        // Scale, and clamp x to 0..1 range
        x = MathUtil.clamp((x - edge0) / (edge1 - edge0), 0, 1);

        return (end - start) * (x * x * x * (x * (6.0f * x - 15.0f) + 10.0f)) + start;
    }

    /**
     * Returns a factor which starts at 1, and decreases to approach 0 as current
     * passes from approachStart to target
     */
    public static double approachFactor(double current, double target, double approachStart) {
        // if in order, current, approachStart, target, either ascending or descending,
        // then we are not in approach
        double approachRange = target - approachStart;
        double dist = target - current;
        if (Math.abs(dist) > Math.abs(approachRange)) // not in approach range yet
            return 1; // continue as we are
        return dist / approachRange;
    }

    public static boolean inAllianceZone(Pose2d robotPose) {
        if (isOnRed()) {
            return robotPose.getMeasureX().gt(Constants.RED_HUB_POS.getMeasureX());
        }
        return robotPose.getMeasureX().lt(Constants.BLUE_HUB_POS.getMeasureX());
    }

    private static boolean teamChecked = false, isOnRed;

    public static boolean isOnRed() {
        if(!teamChecked){
            isOnRed = DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false);
            teamChecked = true;
        }
        return isOnRed;
    }

    public static Translation2d getTargetPosition() {
        Pose2d robotPose = RobotContainer.drivetrain.getPose();

        if (Utils.inAllianceZone(robotPose)) {
            if (isOnRed()) {
                return Constants.RED_HUB_POS;
            }
            return Constants.BLUE_HUB_POS;
        }
        if (isOnRed()) {
            if (robotPose.getMeasureY().gt(Constants.RED_HUB_POS.getMeasureY())) {
                return Constants.RED_RIGHT_SNOWBLOW_TARGET;
            }
            return Constants.RED_LEFT_SNOWBLOW_TARGET;
        }
        if (robotPose.getMeasureY().gt(Constants.BLUE_HUB_POS.getMeasureY())) {
            return Constants.BLUE_LEFT_SNOWBLOW_TARGET;
        }
        return Constants.BLUE_RIGHT_SNOWBLOW_TARGET;
    }
}
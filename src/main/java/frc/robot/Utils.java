// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.util.LookupTable;

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

    //Call before auto to update the team cache
    public static void recheckTeam(){
        teamChecked = false;
        isOnRed();
    }
    
    //Uses the non-moving estimate and factors in the robot's momentum and the estimated travel time of the shot
    public static Translation2d getTargetPosition(){
        Translation2d est = getTargetPositionEstimate();
        //offset based on product of robot velocity and shot travel time
        LinearVelocity robotVelocity = RobotContainer.drivetrain.getVelocity();
        Rotation2d robotMovementDirection = RobotContainer.drivetrain.getMovementDirection();
        //Translation2d offset = new Translation2d(robotVelocity.in(MetersPerSecond), robotMovementDirection);
        Distance dist = Meters.of(RobotContainer.drivetrain.getPose().getTranslation().minus(est).getNorm());
        //Since we likely haven't set the turrets yet, let's use the lookup table for this estimation
        Translation2d turretPos = robotRelativePosition(Constants.BETWEEN_TURRETS_POS.toTranslation2d(), RobotContainer.drivetrain.getPose());
        Translation2d turretToTarget = est.minus(turretPos);
        LookupTable.Result result = Constants.SHOOTER_LOOKUP_TABLE.lookup(Meters.of(turretToTarget.getNorm()));
        Angle hoodAngle = HoodSubsystem.getShootAngle(result.hoodServoSetting());
        //Multiply the meters per second count from the offset by the number of seconds
        Time shotTime = getShotTime(dist, hoodAngle, result.shooterSpeed());
        Distance offsetDist = robotVelocity.times(shotTime);
        RobotContainer.logger.hood_angle_est.set(hoodAngle.in(Degrees));
        RobotContainer.logger.shot_time_est.set(shotTime.in(Seconds));
        RobotContainer.logger.shot_offset.set(offsetDist.in(Meters));
        RobotContainer.logger.turret_pos.set(turretPos);
        //offset = offset.times(getShotTime(dist, hoodAngle, result.shooterSpeed()).in(Seconds));
        return est.plus(new Translation2d(offsetDist.in(Meters), robotMovementDirection));
    }

    public static Translation2d getTargetPositionEstimate() {
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

    /**Estimate the time from when the shot leaves the robot to when it enters the hub. Useful for accounting for robot motion*/
    public static Time getShotTime(Distance distance, Angle shotAngle, AngularVelocity shooterMotorSpeed){
        AngularVelocity shooterWheelSpeed = shooterMotorSpeed.times(10.0/7.0); //should be the gear ratio
        //surface speed = arc length/time = (radius times angle)/time = radius times angular speed in rad/s
        LinearVelocity shotSpeed = InchesPerSecond.of(shooterWheelSpeed.in(RadiansPerSecond) * 1.5); //radius 1.5 inches
        double cos = Math.cos(shotAngle.in(Radians));
        return distance.div(shotSpeed.times(cos));
    }

    public static Translation2d robotRelativePosition(Translation2d offset, Pose2d robotPose){
        offset = offset.rotateAround(Translation2d.kZero, robotPose.getRotation());
        return robotPose.getTranslation().plus(offset);
    }
}
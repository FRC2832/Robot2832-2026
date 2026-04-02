// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LookupTable;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.util.FlippingUtil;

/** Add your docs here. */
public class Constants {
    // General constants
    public static class KrakenX60 {
        public static final AngularVelocity FREE_SPEED = RPM.of(6000);
    }

    // Enable each shooter, PPT, hood, turret
    public static final boolean LEFT_TURRET_ENABLED = true;
    public static final boolean RIGHT_TURRET_ENABLED = true;
    public static final boolean LEFT_HOOD_ENABLED = true;
    public static final boolean RIGHT_HOOD_ENABLED = true;
    public static final boolean LEFT_SHOOTER_ENABLED = true;
    public static final boolean RIGHT_SHOOTER_ENABLED = true;
    public static final boolean LEFT_PPT_ENABLED = true;
    public static final boolean RIGHT_PPT_ENABLED = true;

    // Constants for Hopper subsystem
    public static final int RIGHT_PPT_ID = 25;
    public static final int LEFT_PPT_ID = 15;
    public static final int ACCELERATOR_ID = 27;
    public static final int INDEXER_ID = 17;
    public static final MotorType INDEXER_MOTOR_TYPE = MotorType.kBrushless;

    // Constants for Intake subsystem
    public static final int INTAKE_INTERNAL_ROTATOR_ID = 19;
    public static final int INTAKE_EXTENDER_ID = 18;
    public static final double INTAKE_EXTEND_VOLTAGE = 1.5; // TODO consider raising

    // Constants for Shooter subsytem
    public static final int LEFT_SHOOTER_ID = 14;
    public static final int RIGHT_SHOOTER_ID = 24;

    // Constants for Turret subsystem
    public static final int LEFT_ROTATOR_ID = 16;
    public static final int RIGHT_ROTATOR_ID = 26;
    public static final int LEFT_ROTATOR_CANCODER_ID = 4;
    public static final int RIGHT_ROTATOR_CANCODER_ID = 5;
    public static final int SERVO_HUB_ID = 7; // meant to be 7 but refuses to update in rev hardware client

    public static final double TURRET_GEAR_RATIO = 9d / 1120d;
    public static final double TURRET_ENCODER_RATIO = 8d / 9d; // 140d/180d;
    /** The sensitivity of the manual control for hood angle */
    public static final double HOOD_SENSITIVITY = 0.05;
    public static final double TURRET_MAX_VOLTAGE = .4;
    // TODO update when the turret range increases
    public static final Angle LEFT_TURRET_MIN_ANGLE = Degrees.of(-45);
    public static final Angle LEFT_TURRET_MAX_ANGLE = Degrees.of(45);
    public static final Angle RIGHT_TURRET_MIN_ANGLE = Degrees.of(-45);
    public static final Angle RIGHT_TURRET_MAX_ANGLE = Degrees.of(45);

    public static final Angle LEFT_TURRET_LOW_HARD_STOP = Degrees.of(-50);
    public static final Angle LEFT_TURRET_HIGH_HARD_STOP = Degrees.of(50);
    public static final Angle RIGHT_TURRET_LOW_HARD_STOP = Degrees.of(-50);
    public static final Angle RIGHT_TURRET_HIGH_HARD_STOP = Degrees.of(50);

    public static Angle LEFT_TURRET_ENCODER_OFFSET = Rotations.of(.03217 + .01709 - .02);
    public static Angle RIGHT_TURRET_ENCODER_OFFSET = Rotations.of(-.350801 - .107442 -.014);

    public static final double TURRET_CONTROL_DEADZONE = 0.2;

    // FIXME measure robot turret positions relative to robot origin
    public static final Translation3d LEFT_TURRET_POS = new Translation3d(Inches.of(-6.75), Inches.of(8.75),
            Inches.of(20));
    public static final Translation3d RIGHT_TURRET_POS = new Translation3d(Inches.of(-6.75), Inches.of(-8.75),
            Inches.of(20));
    public static final Translation3d BETWEEN_TURRETS_POS = new Translation3d(Inches.of(-6.75), Inches.of(0), Inches.of(20));
    // Used for approaching the edges of the firing arc slower
    // public static final double LEFT_UPPER_APPROACH_ANGLE =
    // (2*LEFT_TURRET_MAX_ANGLE + LEFT_TURRET_MIN_ANGLE) / 3;
    // public static final double RIGHT_UPPER_APPROACH_ANGLE =
    // (2*RIGHT_TURRET_MAX_ANGLE + RIGHT_TURRET_MIN_ANGLE) / 3;
    // public static final double LEFT_LOWER_APPROACH_ANGLE = (LEFT_TURRET_MAX_ANGLE
    // + 2*LEFT_TURRET_MIN_ANGLE) / 3;
    // public static final double RIGHT_LOWER_APPROACH_ANGLE =
    // (RIGHT_TURRET_MAX_ANGLE + 2*RIGHT_TURRET_MIN_ANGLE) / 3;

    // First word is the hood the servo is on, relative to robot orientation
    // Second direction is the side of that hood the servo is on
    public static final int LEFT_HOOD_LEFT_PORT = 0;
    public static final int LEFT_HOOD_RIGHT_PORT = 1;
    public static final int RIGHT_HOOD_LEFT_PORT = 5;
    public static final int RIGHT_HOOD_RIGHT_PORT = 4;

    // Other CAN ID Constants
    public static final CANBus CANivoreCANBus = new CANBus("CANivore");
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Transform3d LEFT_CAM_POSE = new Transform3d(
            Inches.of(1.1875),
            Inches.of(14.125),
            Inches.of(20.5),
            new Rotation3d(0, -25 * Math.PI / 180, 90 * Math.PI / 180));
    public static final String LEFT_CAM_NAME = "Left Camera";

    public static final Transform3d RIGHT_CAM_POSE = new Transform3d(
            Inches.of(1.1875),
            Inches.of(-14.125),
            Inches.of(20.5),
            // Right camera is 90 degrees clockwise, making the image have to be 90 degrees
            // counterclockwise
            new Rotation3d(0, -25 * Math.PI / 180, -90 * Math.PI / 180));
    public static final String RIGHT_CAM_NAME = "Right Camera";

    public static final Transform3d REAR_CAM_POSE = new Transform3d(
            Inches.of(-11.9375),
            Inches.of(-5),
            Inches.of(11.25),
            new Rotation3d(0, -25 * Math.PI / 180, Math.PI));
    public static final String REAR_CAM_NAME = "Rear Camera";
    
    public static final String FRONT_CAM_NAME = "Front Camera";
    public static final double MAX_AMBIGUITY = 0.8; 

    public static final Translation2d BLUE_HUB_POS = new Translation2d(Inches.of(182.11), Inches.of(158.84));
    public static final Translation2d RED_HUB_POS = FlippingUtil.flipFieldPosition(BLUE_HUB_POS);
    public static final Distance SNOWBLOW_TARGET_OFFSET_Y = Inches.of(70);
    public static final Distance SNOWBLOW_TARGET_OFFSET_X = Inches.of(30);
    public static final Translation2d BLUE_LEFT_SNOWBLOW_TARGET = new Translation2d(BLUE_HUB_POS.getMeasureX()
            .minus(SNOWBLOW_TARGET_OFFSET_X), BLUE_HUB_POS.getMeasureY().plus(SNOWBLOW_TARGET_OFFSET_Y));
    public static final Translation2d BLUE_RIGHT_SNOWBLOW_TARGET = new Translation2d(BLUE_HUB_POS.getMeasureX()
            .minus(SNOWBLOW_TARGET_OFFSET_X), BLUE_HUB_POS.getMeasureY().minus(SNOWBLOW_TARGET_OFFSET_Y));
    public static final Translation2d RED_LEFT_SNOWBLOW_TARGET = new Translation2d(RED_HUB_POS.getMeasureX()
            .plus(SNOWBLOW_TARGET_OFFSET_X), RED_HUB_POS.getMeasureY().minus(SNOWBLOW_TARGET_OFFSET_Y));
    public static final Translation2d RED_RIGHT_SNOWBLOW_TARGET = new Translation2d(RED_HUB_POS.getMeasureX()
            .plus(SNOWBLOW_TARGET_OFFSET_X), RED_HUB_POS.getMeasureY().plus(SNOWBLOW_TARGET_OFFSET_Y));

    public static final LookupTable SHOOTER_LOOKUP_TABLE = new LookupTable(
            new Distance[] { Meters.of(3.131707461), Meters.of(2.905395605), Meters.of(2.207706018),
                    Meters.of(2.214880037), Meters.of(2.236692242), Meters.of(2.061506812), Meters.of(2.810240181),
                    Meters.of(2.836135891), Meters.of(4.944140737), Meters.of(2.704494197), Meters.of(3.096642723),
                    Meters.of(3.348627156), Meters.of(3.214721528), Meters.of(3.339439679), Meters.of(3.218443989),
                    Meters.of(4.710429101), Meters.of(4.55551602), Meters.of(4.112445384), Meters.of(3.909571046),
                    Meters.of(3.343450924), Meters.of(4.708524724), Meters.of(4.545698486), Meters.of(4.088802765),
                    Meters.of(3.93504571), Meters.of(3.360961813)},
            new AngularVelocity[] { RotationsPerSecond.of(50), RotationsPerSecond.of(50), RotationsPerSecond.of(50),
                    RotationsPerSecond.of(55), RotationsPerSecond.of(50), RotationsPerSecond.of(50),
                    RotationsPerSecond.of(50), RotationsPerSecond.of(50), RotationsPerSecond.of(60),
                    RotationsPerSecond.of(50), RotationsPerSecond.of(50), RotationsPerSecond.of(50),
                    RotationsPerSecond.of(55), RotationsPerSecond.of(50), RotationsPerSecond.of(60),
                    RotationsPerSecond.of(65), RotationsPerSecond.of(70), RotationsPerSecond.of(60),
                    RotationsPerSecond.of(55), RotationsPerSecond.of(50), RotationsPerSecond.of(65),
                    RotationsPerSecond.of(70), RotationsPerSecond.of(60), RotationsPerSecond.of(55),
                    RotationsPerSecond.of(50)},
            new double[] { -0.2206255, -0.559337, -0.850459, -0.8642775, -0.8642775, -0.9273405, -0.415452, -0.415452,
                    -0.202339, -0.4796695, -0.019498, -0.022714, -0.7685125, -0.287155, -0.577499, -0.442192, -0.442192,
                    -0.442192, -0.442192, -0.023005, -0.465704, -0.394342, -0.394342, 0, -0.020297});

    public static final boolean SHOULD_AUTO_SET_SPEED_AT_START = true;
    public static final boolean SHOULD_AUTO_AIM_TURRET_AT_START = false;

}
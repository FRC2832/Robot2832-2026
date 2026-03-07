// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.CANBus;




/** Add your docs here. */
public class Constants {
    // General constants
    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
     }

    // Constants for Hopper subsystem
    public static final int RIGHT_PPT_ID = 25;
    public static final int LEFT_PPT_ID = 15;
    public static final int ACCELERATOR_ID = 27;
    public static final int INDEXER_ID = 17;
    public static final MotorType INDEXER_MOTOR_TYPE = MotorType.kBrushless;

    // Constants for Intake subsystem
    public static final int INTAKE_INTERNAL_ROTATOR_ID = 19;
    public static final int INTAKE_EXTENDER_ID = 18;
    

    // Constants for Shooter subsytem
    public static final int LEFT_SHOOTER_ID = 14;
    public static final int RIGHT_SHOOTER_ID = 24;

    // FIXME: Find these values and change it here.
    public static final int LEFT_SERVO_ID = 28326861;
    public static final int RIGHT_SERVO_ID = 28326861; 

    // Constants for Turret subsystem
    public static final int LEFT_ROTATOR_ID = 16;
    public static final int RIGHT_ROTATOR_ID = 26;
    public static final int LEFT_ROTATOR_CANCODER_ID = 4;
    public static final int RIGHT_ROTATOR_CANCODER_ID = 5;

    // Other CAN ID Constants
    public static final CANBus CANivoreCANBus = new CANBus("CANIvore");
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
    public static final Transform3d leftCamPose = new Transform3d(.01905, .31115, .5334, new Rotation3d(0, 25*Math.PI/180, 90*Math.PI/180));
    public static final String leftCamName = "Left Camera";

    public static final Transform3d rightCamPose = new Transform3d(.01905, -.31115, .5334, new Rotation3d(0, 25*Math.PI/180, -90*Math.PI/180));
    public static final String rightCamName = "Right Camera";
}
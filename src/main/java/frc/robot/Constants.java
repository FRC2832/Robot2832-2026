// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;

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

    // Constants for Turret subsystem
    public static final int LEFT_ROTATOR_ID = 16;
    public static final int RIGHT_ROTATOR_ID = 26;
    public static final int LEFT_ROTATOR_CANCODER_ID = 4;
    public static final int RIGHT_ROTATOR_CANCODER_ID = 5;


}

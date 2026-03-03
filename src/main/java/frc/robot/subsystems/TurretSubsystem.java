// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    /** Creates a new TurretSubsystem. */
    private TalonFX leftRotationMotor, rightRotationMotor;
    private CANcoder leftCANcoder, rightCANcoder;

    public TurretSubsystem() {
        leftRotationMotor = new TalonFX(Constants.LEFT_ROTATOR_ID);
        rightRotationMotor = new TalonFX(Constants.RIGHT_ROTATOR_ID);
        leftCANcoder = new CANcoder(Constants.LEFT_ROTATOR_CANCODER_ID);
        rightCANcoder = new CANcoder(Constants.RIGHT_ROTATOR_CANCODER_ID);
    }

    public void setAngle(double angle) {
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

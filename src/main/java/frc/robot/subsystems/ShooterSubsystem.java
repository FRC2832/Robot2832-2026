// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new Shooter. */
    private TalonFX rightShooterMotor, leftShooterMotor;

    public ShooterSubsystem() {
        rightShooterMotor = new TalonFX(Constants.RIGHT_SHOOTER_ID);
        leftShooterMotor = new TalonFX(Constants.LEFT_SHOOTER_ID);
    }

    public void setMotorSpeed(double speed) {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  /** Creates a new HopperSubsystem. */
    private TalonFX rightPPT, leftPPT, accelerator;

  public HopperSubsystem() {
    rightPPT = new TalonFX(Constants.RIGHT_PPT_ID);
    leftPPT = new TalonFX(Constants.LEFT_PPT_ID);
    accelerator = new TalonFX(Constants.ACCELERATOR_ID);
  }

  public void setPPTSpeed(double speed) {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

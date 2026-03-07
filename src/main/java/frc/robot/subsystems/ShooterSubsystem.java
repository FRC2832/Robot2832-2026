// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new Shooter. */
    private TalonFX rightShooterMotor, leftShooterMotor, accelerator;

    public ShooterSubsystem() {
        rightShooterMotor = new TalonFX(Constants.RIGHT_SHOOTER_ID, Constants.CANivoreCANBus);
        leftShooterMotor = new TalonFX(Constants.LEFT_SHOOTER_ID, Constants.CANivoreCANBus);
        accelerator = new TalonFX(Constants.ACCELERATOR_ID, Constants.CANivoreCANBus);

        // TODO: Figure out inversion state of motors
        // Confirm appropriate inversion, voltage limits, current limits, and PID
        // constants
        configureMotor(rightShooterMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(leftShooterMotor, InvertedValue.CounterClockwise_Positive); // inverted
        configureMotor(accelerator, InvertedValue.Clockwise_Positive);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(invertDirection)
                                .withNeutralMode(NeutralModeValue.Coast))
                .withVoltage(
                        new VoltageConfigs()
                                .withPeakReverseVoltage(Volts.of(0)))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(40))
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(0.5)
                                .withKI(2)
                                .withKD(0)
                                .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting
                                                                                            // max RPS
                );
        motor.getConfigurator().apply(config);
    }

    public void setMotorSpeed(double rightSpeed, double leftSpeed, double accelSpeed) {
        rightShooterMotor.set(rightSpeed);
        leftShooterMotor.set(leftSpeed);
        accelerator.set(accelSpeed);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

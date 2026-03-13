// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new Shooter. */
    private TalonFX rightShooterMotor, leftShooterMotor, accelerator;
    public static double leftSpeed = 0.3, rightSpeed = 0.3, acceleratorSpeed = 0.4;

    private VelocityTorqueCurrentFOC control = new VelocityTorqueCurrentFOC(0).withSlot(0);

    public ShooterSubsystem() {
        rightShooterMotor = new TalonFX(Constants.RIGHT_SHOOTER_ID, Constants.CANivoreCANBus);
        leftShooterMotor = new TalonFX(Constants.LEFT_SHOOTER_ID, Constants.CANivoreCANBus);
        accelerator = new TalonFX(Constants.ACCELERATOR_ID, Constants.CANivoreCANBus);

        // TODO Confirm appropriate voltage limits, current limits, and PID
        // constants
        configureMotor(rightShooterMotor, InvertedValue.Clockwise_Positive);
        configureMotor(leftShooterMotor, InvertedValue.CounterClockwise_Positive); // inverted
        configureMotor(accelerator, InvertedValue.CounterClockwise_Positive);

        SmartDashboard.putData(this); 
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
                                .withSupplyCurrentLimit(Amps.of(50))
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(3)
                                .withKI(0.5)
                                .withKD(0.01)
                                .withKS(0.05)
                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                // I changed the KV.
                                .withKV(8.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting
                                                                                            // max RPS
                );
        motor.getConfigurator().apply(config);
    }

    public void setMotorSpeed(double rightSpeed, double leftSpeed, double accelSpeed) {
        rightShooterMotor.setControl(control.withVelocity(KrakenX60.kFreeSpeed.times(rightSpeed)));
        leftShooterMotor.setControl(control.withVelocity(KrakenX60.kFreeSpeed.times(leftSpeed)));
        accelerator.setControl(control.withVelocity(KrakenX60.kFreeSpeed.times(accelSpeed)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public static double getLastLeftSpeed(){
        return leftSpeed;
    }

    public static double getLastRightSpeed(){
        return rightSpeed;
    }

    public static double getLastAcceleratorSpeed(){
        return acceleratorSpeed;
    }

    public Command stopShooter() 
    { 
        return startEnd(
            () -> {
                setMotorSpeed(0, 0, 0);
            },
            () -> {}
        );
    }

    public Command reverseShooter() 
    { 
        return startEnd(
            () -> {
                setMotorSpeed(-.4, -.4, -.4);
            },
            () -> {}
        );
    }
}

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new Shooter. */
    private TalonFX rightShooterMotor, leftShooterMotor, accelerator;
    public static double leftSpeed = 0.28, rightSpeed = 0.28, acceleratorSpeed = 0.33;

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
                                .withSupplyCurrentLimit(Amps.of(55))
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(13)
                                .withKI(0)
                                .withKD(0.1)
                                .withKS(0.05)
                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                // I changed the KV.
                                .withKV(26.31 / 31.277344) //current for rps, over rps
                                .withKA(17.25 / 44.38)
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
        double left, right, acc;
        left = leftShooterMotor.getVelocity().getValueAsDouble();
        right = rightShooterMotor.getVelocity().getValueAsDouble();
        acc = accelerator.getVelocity().getValueAsDouble();
        RobotContainer.logger.leftShooterSpeed.set(left);
        RobotContainer.logger.rightShooterSpeed.set(right);
        RobotContainer.logger.acceleratorSpeed.set(acc);
        RobotContainer.logger.leftShooterAtSpeed.set(MathUtil.isNear(left, 28, 1));
        RobotContainer.logger.rightShooterAtSpeed.set(MathUtil.isNear(right, 28, 1));
        RobotContainer.logger.acceleratorAtSpeed.set(acc > 32);
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
                leftShooterMotor.set(0);
                rightShooterMotor.set(0);
                accelerator.set(0);
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

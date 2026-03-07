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
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;

public class TurretSubsystem extends SubsystemBase {
    /** Creates a new TurretSubsystem. */
    private TalonFX leftRotationMotor, rightRotationMotor;
    private CANcoder leftCANcoder, rightCANcoder;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    public TurretSubsystem() {
        leftRotationMotor = new TalonFX(Constants.LEFT_ROTATOR_ID, Constants.CANivoreCANBus);
        rightRotationMotor = new TalonFX(Constants.RIGHT_ROTATOR_ID, Constants.CANivoreCANBus);
        leftCANcoder = new CANcoder(Constants.LEFT_ROTATOR_CANCODER_ID, Constants.CANivoreCANBus);
        rightCANcoder = new CANcoder(Constants.RIGHT_ROTATOR_CANCODER_ID, Constants.CANivoreCANBus);

        leftRotationMotor.setPosition(Angle.ofBaseUnits(0, Units.Degrees));
        rightRotationMotor.setPosition(Angle.ofBaseUnits(0, Units.Degrees));

        // TODO: Figure out inversion state of motors
        // Confirm appropriate inversion, voltage limits, current limits, and PID constants
        configureMotor(leftRotationMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(rightRotationMotor, InvertedValue.CounterClockwise_Positive); // inverted
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
            motor.getConfigurator().apply(config);
    }

    public void setAngle(double leftAngle, double rightAngle) {
        leftRotationMotor.setControl(motionMagicRequest.withPosition(Angle.ofRelativeUnits(leftAngle, Units.Degrees)));
        rightRotationMotor.setControl(motionMagicRequest.withPosition(Angle.ofRelativeUnits(rightAngle, Units.Degrees)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

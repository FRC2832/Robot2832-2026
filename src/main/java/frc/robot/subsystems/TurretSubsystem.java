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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;

public class TurretSubsystem extends SubsystemBase {
    /** Creates a new TurretSubsystem. */
    private TalonFX leftRotationMotor, rightRotationMotor;
    private CANcoder leftCANcoder, rightCANcoder;
    //First word is the hood the servo is on, relative to robot orientation
    private Servo leftHoodLeftServo, leftHoodRightServo, rightHoodLeftServo, rightHoodRightServo;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    private double leftHoodPosition, rightHoodPosition;

    public static boolean autoAim = false;

    /**The sensitivity of the manual control for hood angle*/
    private static final double HOOD_SENSITIVITY = 0.1;

    public TurretSubsystem() {
        leftRotationMotor = new TalonFX(Constants.LEFT_ROTATOR_ID, Constants.CANivoreCANBus);
        rightRotationMotor = new TalonFX(Constants.RIGHT_ROTATOR_ID, Constants.CANivoreCANBus);
        leftCANcoder = new CANcoder(Constants.LEFT_ROTATOR_CANCODER_ID, Constants.CANivoreCANBus);
        rightCANcoder = new CANcoder(Constants.RIGHT_ROTATOR_CANCODER_ID, Constants.CANivoreCANBus);
        leftHoodLeftServo = new Servo(Constants.LEFT_HOOD_LEFT_PORT);
        leftHoodRightServo = new Servo(Constants.LEFT_HOOD_RIGHT_PORT);
        rightHoodLeftServo = new Servo(Constants.RIGHT_HOOD_LEFT_PORT);
        rightHoodRightServo = new Servo(Constants.RIGHT_HOOD_RIGHT_PORT);
        leftHoodLeftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        leftHoodRightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightHoodLeftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightHoodRightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

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
        //TODO update when the turret range increases
        rightAngle = MathUtil.clamp(rightAngle, 0, 90);
        leftAngle = MathUtil.clamp(leftAngle, -90, 0);
        leftRotationMotor.setControl(motionMagicRequest.withPosition(Angle.ofRelativeUnits(leftAngle, Units.Rotations)));
        rightRotationMotor.setControl(motionMagicRequest.withPosition(Angle.ofRelativeUnits(rightAngle, Units.Rotations)));
    }

    public void setTurretVoltage(double leftVoltage, double rightVoltage){
        leftVoltage = MathUtil.clamp(leftVoltage, -12, 12);
        rightVoltage = MathUtil.clamp(rightVoltage, -12, 12);
        leftRotationMotor.setVoltage(leftVoltage);
        rightRotationMotor.setVoltage(rightVoltage);
    }

    public void setHoodPositions(double leftHoodPosition, double rightHoodPosition){
        this.leftHoodPosition = leftHoodPosition;
        this.rightHoodPosition = rightHoodPosition;
        leftHoodLeftServo.set(leftHoodPosition);
        leftHoodRightServo.set(leftHoodPosition);
        rightHoodLeftServo.set(rightHoodPosition);
        rightHoodRightServo.set(rightHoodPosition);
    }

    public double getLeftHoodPosition(){
        return leftHoodPosition;
    }

    public double getRightHoodPosition(){
        return rightHoodPosition;
    }

    public void setHoodSpeeds(double leftHoodSpeed, double rightHoodSpeed){
        leftHoodPosition += HOOD_SENSITIVITY * leftHoodSpeed;
        rightHoodPosition += HOOD_SENSITIVITY * rightHoodSpeed;
        leftHoodLeftServo.set(leftHoodPosition);
        leftHoodRightServo.set(leftHoodPosition);
        rightHoodLeftServo.set(rightHoodPosition);
        rightHoodRightServo.set(rightHoodPosition);
    }

    @Override
    public void periodic() {
        
    }
}

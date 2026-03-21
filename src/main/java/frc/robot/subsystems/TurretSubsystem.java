// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.MissingFormatArgumentException;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Constants.KrakenX60;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
    /** Creates a new TurretSubsystem. */
    private TalonFX leftRotationMotor, rightRotationMotor;
    private CANcoder leftCANcoder, rightCANcoder;
    //First word is the hood the servo is on, relative to robot orientation
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    private TalonFXWrapper leftRotationMotorWrapper, rightRotationMotorWrapper;

    private Pivot leftTurret, rightTurret;

    private double turretTargetAngle = 0;

    public static boolean isAutoAim = false;

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

        leftRotationMotorWrapper = configureSmartMotor(leftRotationMotor, InvertedValue.CounterClockwise_Positive, leftCANcoder);
        rightRotationMotorWrapper = configureSmartMotor(rightRotationMotor, InvertedValue.CounterClockwise_Positive, rightCANcoder);

        leftTurret = configurePivot(leftRotationMotorWrapper);
        rightTurret = configurePivot(rightRotationMotorWrapper);

        SmartDashboard.putData(this); 
    }

    private TalonFXWrapper configureSmartMotor(TalonFX motor, InvertedValue invertDirection, CANcoder cancoder){
        return new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), 
            new SmartMotorControllerConfig(this)
                .withGearing(new MechanismGearing(GearBox.fromStages("1:10", "30:40", "15:140")))
                .withExternalEncoder(cancoder)
                .withExternalEncoderGearing(new MechanismGearing(GearBox.fromStages("12:1", "15:140")))
                .withVendorConfig(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs()
                                .withInverted(invertDirection)
                                .withNeutralMode(NeutralModeValue.Coast)
                        )
                        // .withVoltage(
                        //     new VoltageConfigs()
                        //         .withPeakReverseVoltage(Volts.of(0))
                        // )
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
                        )
                    )
        );
    }

    private Pivot configurePivot(SmartMotorController motorWrapper){
        PivotConfig config = new PivotConfig(motorWrapper)
                .withHardLimit(Degrees.of(-90), Degrees.of(90));
        return new Pivot(config);
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

    public void setTurretPositions(Angle left, Angle right){
        leftTurret.setAngle(left);
        rightTurret.setAngle(right);
    }

    public void setTurretRotationSpeed(double left, double right){
        leftTurret.setVoltage(Volts.of(left * 3));
        rightTurret.setVoltage(Volts.of(right * 3));
    }

    public double getLeftTurretAngle(){
        return 1/Constants.TURRET_ENCODER_RATIO * leftCANcoder.getAbsolutePosition().getValue().in(Degrees);
    }

    public double getRightTurretAngle(){
        return 1/Constants.TURRET_ENCODER_RATIO * rightCANcoder.getAbsolutePosition().getValue().in(Degrees);
    }

    public double getLeftMotorAngle(){
        return leftRotationMotor.getPosition().getValue().in(Degrees);
    }

    public double getRightMotorAngle(){
        return rightRotationMotor.getPosition().getValue().in(Degrees);
    }

    public void setTurretAngle(double leftAngle, double rightAngle) {
        rightAngle = MathUtil.clamp(rightAngle, Constants.RIGHT_TURRET_MIN_ANGLE, Constants.RIGHT_TURRET_MAX_ANGLE);
        leftAngle = MathUtil.clamp(leftAngle, Constants.LEFT_TURRET_MIN_ANGLE, Constants.LEFT_TURRET_MAX_ANGLE);
        leftRotationMotor.setPosition(Angle.ofRelativeUnits(leftAngle, Units.Degrees));
        rightRotationMotor.setPosition(Angle.ofRelativeUnits(rightAngle, Units.Degrees));
    }

    public void moveTurretTargetAngle(double offset){
        turretTargetAngle += offset;
        double leftTarget = MathUtil.clamp(turretTargetAngle, Constants.LEFT_TURRET_MIN_ANGLE, Constants.LEFT_TURRET_MAX_ANGLE);
        double rightTarget = MathUtil.clamp(turretTargetAngle, Constants.RIGHT_TURRET_MIN_ANGLE, Constants.LEFT_TURRET_MAX_ANGLE);
        leftRotationMotor.setPosition(Angle.ofRelativeUnits(leftTarget, Units.Degrees));
        rightRotationMotor.setPosition(Angle.ofRelativeUnits(rightTarget, Units.Degrees));
    }

    public void setTurretVoltage(double leftVoltage, double rightVoltage){
        leftVoltage = MathUtil.clamp(leftVoltage, -12, 12);
        rightVoltage = MathUtil.clamp(rightVoltage, -12, 12);
        double leftAngle = getLeftTurretAngle();
        if(leftAngle < Constants.LEFT_TURRET_MIN_ANGLE){
            //leftRotationMotor.setControl(motionMagicRequest.withPosition(Degrees.of(Constants.LEFT_TURRET_MIN_ANGLE)));
        }else if(leftAngle > Constants.LEFT_TURRET_MAX_ANGLE){
            //leftRotationMotor.setControl(motionMagicRequest.withPosition(Degrees.of(Constants.LEFT_TURRET_MAX_ANGLE)));
        }else{
            leftRotationMotor.setVoltage(leftVoltage);
        }
        double rightAngle = getRightTurretAngle();
        if(rightAngle < Constants.RIGHT_TURRET_MIN_ANGLE * 1.1){
            rightVoltage = MathUtil.clamp(rightVoltage, -12, 0);
        }else if(rightAngle > Constants.RIGHT_TURRET_MAX_ANGLE * 1.1){
            rightVoltage = MathUtil.clamp(rightVoltage, 0, 12);
        }
        rightRotationMotor.setVoltage(rightVoltage);
        // double leftFactor = Utils.approachFactor(leftAngle, Constants.LEFT_TURRET_MAX_ANGLE, Constants.LEFT_UPPER_APPROACH_ANGLE) 
        //             * Utils.approachFactor(leftAngle, Constants.LEFT_TURRET_MIN_ANGLE, Constants.LEFT_LOWER_APPROACH_ANGLE);
        // double rightFactor = Utils.approachFactor(rightAngle, Constants.RIGHT_TURRET_MAX_ANGLE, Constants.RIGHT_UPPER_APPROACH_ANGLE) 
        //             * Utils.approachFactor(rightAngle, Constants.RIGHT_TURRET_MIN_ANGLE, Constants.RIGHT_LOWER_APPROACH_ANGLE);
    }

    public Command stopTurretRotation(){
        return runOnce(() -> setTurretVoltage(0, 0));
    }

    @Override
    public void periodic() {
        
    }
}

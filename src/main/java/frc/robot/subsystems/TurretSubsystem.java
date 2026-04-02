// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
    TalonFX motor;
    CANcoder cancoder;
    TalonFXWrapper motorWrapper;
    Pivot turret;
    Angle minAngle, maxAngle;
    Translation3d position;
    boolean isLeft;

    public boolean isAutoAim;

    PositionVoltage positionControl = new PositionVoltage(0);

    public TurretSubsystem(int motor_id, int encoder_id, InvertedValue inverted, Angle minAngle, Angle maxAngle,
            boolean isLeft) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        motor = new TalonFX(motor_id, Constants.CANivoreCANBus);
        cancoder = new CANcoder(encoder_id, Constants.CANivoreCANBus);
        motorWrapper = configureSmartMotor(motor, inverted, cancoder, isLeft);
        turret = configurePivot(motorWrapper);
        this.isLeft = isLeft;
        position = isLeft ? Constants.LEFT_TURRET_POS : Constants.RIGHT_TURRET_POS;
        isAutoAim = false;
    }

    private TalonFXWrapper configureSmartMotor(TalonFX motor, InvertedValue invertDirection, CANcoder cancoder,
            boolean isLeft) {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(invertDirection)
                .withNeutralMode(NeutralModeValue.Coast);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(40))
                .withSupplyCurrentLimitEnable(true);
        Slot0Configs slotConfigs = new Slot0Configs()
                .withKP(0.5)
                .withKI(2)
                .withKD(0)
                // 6 volts gave 0.18 rps
                .withKV(6.0 / 0.18);
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs)
                .withCurrentLimits(currentLimits)
                .withSlot0(slotConfigs);
        return new TalonFXWrapper(motor, DCMotor.getKrakenX60(1),
                new SmartMotorControllerConfig(this)
                        .withGearing(new MechanismGearing(GearBox.fromStages("10:1", "40:30", "140:15")))
                        .withExternalEncoder(cancoder)
                        .withExternalEncoderGearing(
                                new MechanismGearing(GearBox.fromReductionStages(1 / Constants.TURRET_ENCODER_RATIO)))
                        .withExternalEncoderZeroOffset(
                                isLeft ? Constants.LEFT_TURRET_ENCODER_OFFSET : Constants.RIGHT_TURRET_ENCODER_OFFSET)
                        .withExternalEncoderInverted(true)
                        .withUseExternalFeedbackEncoder(true)
                        .withSoftLimit(minAngle, maxAngle)
                        .withVendorConfig(config)
                        .withVendorControlRequest(positionControl)
                        .withStatorCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withMotorInverted(invertDirection != InvertedValue.Clockwise_Positive)
                        .withClosedLoopController(1, 0, 0)
                        .withResetPreviousConfig(true));

    }

    private Pivot configurePivot(SmartMotorController motorWrapper) {
        Angle min = isLeftTurret() ? Constants.LEFT_TURRET_LOW_HARD_STOP : Constants.RIGHT_TURRET_LOW_HARD_STOP;
        Angle max = isLeftTurret() ? Constants.LEFT_TURRET_HIGH_HARD_STOP : Constants.RIGHT_TURRET_HIGH_HARD_STOP;
        PivotConfig config = new PivotConfig(motorWrapper)
                .withHardLimit(min, max);
        return new Pivot(config);
    }

    public void stopRotation() {
        turret.setVoltageSetpoint(Volts.of(0));
    }

    public void setVoltage(Voltage voltage) {
        if (isTurretEnabled())
            turret.setVoltageSetpoint(voltage);
        else
            stopRotation();
    }

    public void setTurretAngle(Angle angle) {
        if (angle.lt(minAngle))
            angle = minAngle;
        if (angle.gt(maxAngle))
            angle = maxAngle;
        if (isTurretEnabled()) {
            turret.setMechanismPositionSetpoint(angle);
        } else
            stopRotation();
    }

    public Angle getTurretAngle() {
        return turret.getAngle();
    }

    public Voltage getTurretVoltage() {
        return motorWrapper.getVoltage();
    }

    public boolean isLeftTurret() {
        return isLeft;
    }

    public void aimAtPosition(Translation2d target, Pose2d robotPose) {
        if (!isTurretEnabled())
            return;
        Pose2d targetPose = new Pose2d(target, Rotation2d.kZero);
        Pose2d turretPose = robotPose.plus(new Transform2d(position.toTranslation2d(), Rotation2d.kZero));
        Pose2d offset = targetPose.relativeTo(turretPose);
        System.out.println("Hub at position " + offset.getX() + "," + offset.getY() + " and rotation "
                + offset.getTranslation().getAngle().getMeasure().in(Degrees) + " relative to "
                + (isLeftTurret() ? "left turret" : "right turret"));
        Angle angle = Degrees.of(180).plus(offset.getTranslation().getAngle().getMeasure());
        angle = Radians.of(MathUtil.angleModulus(angle.in(Radians)));
        System.out.println(
                "Aiming " + (isLeftTurret() ? "left" : "right") + " turret at angle " + angle.in(Degrees) + " degrees");
        setTurretAngle(angle);
        System.out.println((isLeftTurret() ? "Left" : "Right") + " turret currently at angle "
                + turret.getAngle().in(Degrees) + " degrees");
    }

    

    public Command enableAutoAim() {
        return runOnce(() -> this.isAutoAim = true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private boolean isTurretEnabled() {
        return isLeft ? Constants.LEFT_TURRET_ENABLED : Constants.RIGHT_TURRET_ENABLED;
    }

    
}

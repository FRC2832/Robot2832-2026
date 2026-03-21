// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.Constants.KrakenX60;
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

    public TurretSubsystem(int motor_id, int encoder_id, InvertedValue inverted, Angle minAngle, Angle maxAngle,
            boolean isLeft) {
        motor = new TalonFX(motor_id, Constants.CANivoreCANBus);
        cancoder = new CANcoder(encoder_id, Constants.CANivoreCANBus);
        motorWrapper = configureSmartMotor(motor, inverted, cancoder);
        turret = configurePivot(motorWrapper);
        this.isLeft = isLeft;
        position = isLeft ? Constants.LEFT_TURRET_POS : Constants.RIGHT_TURRET_POS;
        isAutoAim = true;
    }

    private TalonFXWrapper configureSmartMotor(TalonFX motor, InvertedValue invertDirection, CANcoder cancoder) {

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
                // 12 volts when requesting max RPS
                .withKV(12.0 / KrakenX60.FREE_SPEED.in(RotationsPerSecond));
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs)
                .withCurrentLimits(currentLimits)
                .withSlot0(slotConfigs);
        return new TalonFXWrapper(motor, DCMotor.getKrakenX60(1),
                new SmartMotorControllerConfig(this)
                        .withGearing(new MechanismGearing(GearBox.fromStages("1:10", "30:40", "15:140")))
                        .withExternalEncoder(cancoder)
                        .withExternalEncoderGearing(
                                new MechanismGearing(GearBox.fromReductionStages(1 / Constants.TURRET_ENCODER_RATIO)))
                        .withSoftLimit(minAngle, maxAngle)
                        .withVendorConfig(config));

    }

    private Pivot configurePivot(SmartMotorController motorWrapper) {
        PivotConfig config = new PivotConfig(motorWrapper)
                .withHardLimit(Degrees.of(-120), Degrees.of(120));
        return new Pivot(config);
    }

    public Command stopRotation() {
        return turret.setVoltage(Volts.of(0));
    }

    public Command setVoltage(Supplier<Voltage> voltage) {
        return turret.setVoltage(voltage);
    }

    public Command setTurretAngle(Supplier<Angle> angles) {
        return turret.setAngle(angles);
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

    public Command aimAtPosition(Supplier<Translation2d> target, Supplier<Pose2d> robotPose) {
        return setTurretAngle(() -> {
            Pose2d targetPose = new Pose2d(target.get(), Rotation2d.kZero);
            Pose2d turretPose = robotPose.get().plus(new Transform2d(position.toTranslation2d(), Rotation2d.kZero));
            Pose2d offset = targetPose.relativeTo(turretPose);
            return Degrees.of(90).minus(offset.getTranslation().getAngle().getMeasure());
        });
    }

    public Command enableAutoAim() {
        return runOnce(() -> this.isAutoAim = true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

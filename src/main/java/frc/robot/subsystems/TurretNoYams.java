// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.RobotContainer;

public class TurretNoYams extends SubsystemBase {
    TalonFX motor;
    CANcoder canCoder;

    boolean isLeft;
    Translation3d position;

    public boolean isAutoAim = Constants.SHOULD_AUTO_AIM_TURRET_AT_START;

    PositionVoltage angleControl = new PositionVoltage(0)
            .withSlot(0);//.withEnableFOC(true);
    VoltageOut voltageControl = new VoltageOut(0);//.withEnableFOC(true);

    public TurretNoYams(boolean isLeft, InvertedValue motorInverted, SensorDirectionValue cancoderDirection, Angle cancoderOffset) {
        RobotContainer.logger.leftTurretAutoAiming.set(isAutoAim);
        RobotContainer.logger.rightTurretAutoAiming.set(isAutoAim);
        this.isLeft = isLeft;
        if(isLeft){
            position = Constants.LEFT_TURRET_POS;
            motor = new TalonFX(Constants.LEFT_ROTATOR_ID, Constants.CANivoreCANBus);
            canCoder = new CANcoder(Constants.LEFT_ROTATOR_CANCODER_ID, Constants.CANivoreCANBus);
        }else{
            position = Constants.RIGHT_TURRET_POS;
            motor = new TalonFX(Constants.RIGHT_ROTATOR_ID, Constants.CANivoreCANBus);
            canCoder = new CANcoder(Constants.RIGHT_ROTATOR_CANCODER_ID, Constants.CANivoreCANBus);
        }
        configureCancoder(canCoder, cancoderDirection, cancoderOffset);
        configureMotor(motor, motorInverted);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
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
                    .withKP(2)
                    .withKI(1)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.FREE_SPEED.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            ).withFeedback(
                new FeedbackConfigs()
                .withRemoteCANcoder(canCoder) //TODO do the below and swap to FusedCANcoder
                //.withRotorToSensorRatio(-1.12) //TODO improve ratio, zero talon at zero encoder angle
                .withSensorToMechanismRatio(Constants.TURRET_ENCODER_RATIO)
            ).withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(isLeft ? Constants.LEFT_TURRET_HIGH_HARD_STOP : Constants.RIGHT_TURRET_HIGH_HARD_STOP)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(isLeft ? Constants.LEFT_TURRET_LOW_HARD_STOP : Constants.RIGHT_TURRET_LOW_HARD_STOP)
            );
            motor.getConfigurator().apply(config);
    }

    private void configureCancoder(CANcoder cancoder, SensorDirectionValue direction, Angle offset){
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withMagnetOffset(offset)
                .withSensorDirection(direction);
        CANcoderConfiguration config = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
        cancoder.getConfigurator().apply(config);
    }

    public void setAngle(Angle targetAngle){
        Angle low = isLeftTurret() ? Constants.LEFT_TURRET_MIN_ANGLE : Constants.RIGHT_TURRET_MIN_ANGLE;
        Angle high = isLeftTurret() ? Constants.LEFT_TURRET_MAX_ANGLE : Constants.RIGHT_TURRET_MAX_ANGLE;
        if(targetAngle.lt(low))
            targetAngle = low;
        else if(targetAngle.gt(high)){
            targetAngle = high;
        }
        motor.setControl(angleControl.withPosition(targetAngle));
    }

    public Angle getAngle(){
        return canCoder.getPosition().getValue();
    }

    public void setVoltage(Voltage volts){
        Angle low = isLeftTurret() ? Constants.LEFT_TURRET_MIN_ANGLE : Constants.RIGHT_TURRET_MIN_ANGLE;
        Angle high = isLeftTurret() ? Constants.LEFT_TURRET_MAX_ANGLE : Constants.RIGHT_TURRET_MAX_ANGLE;
        if(getAngle().lt(low) && volts.lt(Volts.zero())){
            volts = Volts.zero();
        }else if(getAngle().gt(high) && volts.gt(Volts.zero())){
            volts = Volts.zero();
        }
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void aimAtPosition(Translation2d target, Pose2d robotPose) {
        if (!isTurretEnabled())
            return;
        Pose2d targetPose = new Pose2d(target, Rotation2d.kZero);
        Pose2d turretPose = robotPose.plus(new Transform2d(position.toTranslation2d(), Rotation2d.kZero));
        Pose2d offset = targetPose.relativeTo(turretPose);
        // System.out.println("Hub at position " + offset.getX() + "," + offset.getY() + " and rotation "
        //         + offset.getTranslation().getAngle().getMeasure().in(Degrees) + " relative to "
        //         + (isLeftTurret() ? "left turret" : "right turret"));
        Angle angle = Degrees.of(180).plus(offset.getTranslation().getAngle().getMeasure());
        angle = Radians.of(MathUtil.angleModulus(angle.in(Radians)));
        if(isLeftTurret())
            RobotContainer.logger.leftTurretTarget.set(angle.in(Degrees));
        else
            RobotContainer.logger.rightTurretTarget.set(angle.in(Degrees));
        // System.out.println(
        //         "Aiming " + (isLeftTurret() ? "left" : "right") + " turret at angle " + angle.in(Degrees) + " degrees");
        setAngle(angle);
        // System.out.println((isLeftTurret() ? "Left" : "Right") + " turret currently at angle "
        //         + getAngle().in(Degrees) + " degrees");
    }

    public boolean isTurretEnabled(){
        if(isLeftTurret())
            return Constants.LEFT_TURRET_ENABLED;
        return Constants.RIGHT_TURRET_ENABLED;
    }

    public boolean isLeftTurret(){
        return isLeft;
    }
}

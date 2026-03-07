// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */

    // ENUMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -------------------------
     public enum Speed {
        STOP(0),
        INTAKE(0.8),
        REVERSE(-0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        // FIXME: This is wrong - See what the real life values should be. Genuinely do not run the code like this it will NOT be pretty. VS code only has todo but this is like a death siren.
        // HOMED(110),
        STOWED(100),
        INTAKE(-4);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    // --------------------------------------------------
    // Motor setup --------------------------------------

    private TalonFX intakeInternalRotatorMotor, intakeExtenderMotor;

    private final MotionMagicVoltage extenderMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    public IntakeSubsystem() {
        intakeInternalRotatorMotor = new TalonFX(Constants.INTAKE_INTERNAL_ROTATOR_ID, Constants.CANivoreCANBus);
        intakeExtenderMotor = new TalonFX(Constants.INTAKE_EXTENDER_ID, Constants.CANivoreCANBus);

        // TODO: Figure out inversion state of motors
        // Confirm appropriate inversion, voltage limits, current limits, and PID constants
        configureMotor(intakeInternalRotatorMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(intakeExtenderMotor, InvertedValue.CounterClockwise_Positive); // inverted

        SmartDashboard.putData(this); 
    }

    // TODO: These should probably be changed eventually
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

    // ------------------------------------------------------------------
    // ACTUAL FUNCTIONS -------------------------------------------------

    // QUESTION: Do we want a manual override for intake position ????????
    public void setIntakePosition(Position position) {
        intakeExtenderMotor.setControl(
            extenderMotionMagicRequest
                .withPosition(position.angle())
        );
    }

    public void setIntakeSpeed(Speed speed) {
        intakeInternalRotatorMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }

    // ----------------------------------------------------------------------------------
    // COMMANDS -------------------------------------------------------------------------
    
    public Command extendIntakeCommand() {
        return startEnd(
            () -> {
                setIntakePosition(Position.INTAKE);
            },
            () -> setIntakePosition(Position.INTAKE) // what is the pass equivalent. return isn't working. can I just leave it empty.
        );
    }

    public Command retractIntakeCommand() {
        return startEnd(
            () -> {
                setIntakePosition(Position.STOWED);
            },
            () -> setIntakePosition(Position.STOWED)
        );
    }

    public Command runIntakeCommand() {
        return startEnd(
            () -> {
                setIntakeSpeed(Speed.INTAKE);
            },
            () -> setIntakeSpeed(Speed.STOP)
        );
    }

    public Command reverseIntakeCommand() {
        return startEnd(
            () -> {
                setIntakeSpeed(Speed.REVERSE);
            },
            () -> setIntakeSpeed(Speed.STOP)
        );
    }
    // -----------------------------------------------------------------------------
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
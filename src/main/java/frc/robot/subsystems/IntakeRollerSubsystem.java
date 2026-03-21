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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;

public class IntakeRollerSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */

    // ENUMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // -------------------------
    public enum Speed {
        STOP(0),
        INTAKE(0.67),
        REVERSE(-0.67);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    // --------------------------------------------------
    // Motor setup --------------------------------------

    public TalonFX intakeInternalRotatorMotor;

    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    public IntakeRollerSubsystem() {
        intakeInternalRotatorMotor = new TalonFX(Constants.INTAKE_INTERNAL_ROTATOR_ID, Constants.CANivoreCANBus);
        // Confirm appropriate inversion, voltage limits, current limits, and PID
        // constants
        configureMotor(intakeInternalRotatorMotor, InvertedValue.Clockwise_Positive);

        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(invertDirection)
                                .withNeutralMode(NeutralModeValue.Coast))
                // .withVoltage(
                // new VoltageConfigs()
                // .withPeakReverseVoltage(Volts.of(0))
                // )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(50))
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(0.5)
                                .withKI(2)
                                .withKD(0)
                                .withKV(10.0 / KrakenX60.FREE_SPEED.in(RotationsPerSecond)) // 12 volts when requesting
                                                                                            // max RPS
                );
        motor.getConfigurator().apply(config);
    }

    // ------------------------------------------------------------------
    // ACTUAL FUNCTIONS -------------------------------------------------

    public void setIntakeSpeed(Speed speed) {
        intakeInternalRotatorMotor.setControl(
                rollerVoltageRequest
                        .withOutput(speed.voltage()));
    }

    // ----------------------------------------------------------------------------------
    // COMMANDS
    // -------------------------------------------------------------------------

    public Command runIntakeCommand() {
        return startEnd(
                () -> setIntakeSpeed(Speed.INTAKE),
                () -> setIntakeSpeed(Speed.STOP));
    }

    public Command reverseIntakeCommand() {
        return startEnd(
                () -> setIntakeSpeed(Speed.REVERSE),
                () -> setIntakeSpeed(Speed.STOP));
    }

    // -----------------------------------------------------------------------------
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
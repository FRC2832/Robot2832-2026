// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.commands.MoveIntake;

public class IntakeExtenderSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */

    // Motor setup --------------------------------------

    public TalonFX intakeExtenderMotor;

    public IntakeExtenderSubsystem() {
        intakeExtenderMotor = new TalonFX(Constants.INTAKE_EXTENDER_ID, Constants.CANivoreCANBus);
        // Confirm appropriate inversion, voltage limits, current limits, and PID
        // constants
        configureMotor(intakeExtenderMotor, InvertedValue.CounterClockwise_Positive);

        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(invertDirection)
                                .withNeutralMode(NeutralModeValue.Brake))
                // .withVoltage(
                // new VoltageConfigs()
                // .withPeakReverseVoltage(Volts.of(0))
                // )
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(40))
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(0.5)
                                .withKI(2)
                                .withKD(0)
                                .withKV(12.0 / KrakenX60.FREE_SPEED.in(RotationsPerSecond)) // 12 volts when requesting
                                                                                            // max RPS
                );
        motor.getConfigurator().apply(config);
    }

    // ------------------------------------------------------------------
    // COMMANDS
    // -------------------------------------------------------------------------

    public Command extendIntakeCommand() {
        return new MoveIntake(true);
    }

    public Command retractIntakeCommand() {
        return new MoveIntake(false);
    }

    // -----------------------------------------------------------------------------
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
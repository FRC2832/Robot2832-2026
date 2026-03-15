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

public class PPTSubsystem extends SubsystemBase {
    /** Creates a new HopperSubsystem. */

    // ENUMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -------------------------

    public enum Speed {
        STOP(0),
        FORWARD(0.8),
        REVERSE(-0.67);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    // -------------------------------------------------------

    private TalonFX rightPPT, leftPPT;

    private final VoltageOut rightPPTVoltageRequest = new VoltageOut(0);
    private final VoltageOut leftPPTVoltageRequest = new VoltageOut(0);

    public PPTSubsystem() {
        rightPPT = new TalonFX(Constants.RIGHT_PPT_ID, Constants.CANivoreCANBus); 
        leftPPT = new TalonFX(Constants.LEFT_PPT_ID, Constants.CANivoreCANBus);

        // TODO: Figure out inversion state of motors
        // Confirm appropriate inversion, voltage limits, current limits, and PID constants
        configureMotor(rightPPT, InvertedValue.Clockwise_Positive);
        configureMotor(leftPPT, InvertedValue.CounterClockwise_Positive); // inverted

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
            );
            motor.getConfigurator().apply(config);
    }

    // Speed controls ----------------------------------
    public void setPPTSpeed(Speed rightSpeed, Speed leftSpeed) {
        rightPPT.setControl(
            rightPPTVoltageRequest
                .withOutput(rightSpeed.voltage())
        );
        leftPPT.setControl(
            leftPPTVoltageRequest
                .withOutput(leftSpeed.voltage())
        );
    }

    // ---------------------------------------------------
    // COMMANDS ------------------------------------------

    public Command deliverCommand() {
        return startEnd(
            () -> setPPTSpeed(Speed.FORWARD, Speed.FORWARD),
            () -> setPPTSpeed(Speed.STOP,Speed.STOP)
        );
    }

    public Command reverseDeliverCommand() {
        return startEnd(
            () -> setPPTSpeed(Speed.REVERSE, Speed.REVERSE),
            () -> setPPTSpeed(Speed.STOP,Speed.STOP)
        );
    }


    //--------------------------------------------------------




    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

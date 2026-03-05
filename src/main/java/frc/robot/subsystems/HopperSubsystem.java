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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;

public class HopperSubsystem extends SubsystemBase {
    /** Creates a new HopperSubsystem. */

    // ENUMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -------------------------
    public enum IndexerSpeed {
        STOP(0),
        FORWARD(.8),
        REVERSE(-.8);

        private final double percentOutput;

        private IndexerSpeed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum PPTSpeed {
        STOP(0),
        FORWARD(.8),
        REVERSE(-.8);

        private final double percentOutput;

        private PPTSpeed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    // -------------------------------------------------------

    private TalonFX rightPPT, leftPPT;
    private SparkFlex indexer;

    private final VoltageOut rightPPTVoltageRequest = new VoltageOut(0);
    private final VoltageOut leftPPTVoltageRequest = new VoltageOut(0);

    public HopperSubsystem() {
        rightPPT = new TalonFX(Constants.RIGHT_PPT_ID, Constants.CANivoreCANBus); 
        leftPPT = new TalonFX(Constants.LEFT_PPT_ID, Constants.CANivoreCANBus);

        indexer = new SparkFlex(Constants.INDEXER_ID, Constants.INDEXER_MOTOR_TYPE); // I don't configure this motor.

        // TODO: Figure out inversion state of motors
        // Confirm appropriate inversion, voltage limits, current limits, and PID constants
        configureMotor(rightPPT, InvertedValue.CounterClockwise_Positive);
        configureMotor(leftPPT, InvertedValue.CounterClockwise_Positive); // inverted
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

    // Speed controls ----------------------------------
    /** Sets the speed of BOTH PPT motors. */ 
    public void setPPTSpeed(PPTSpeed speed) {
        rightPPT.setControl(
            rightPPTVoltageRequest
                .withOutput(speed.voltage())
        );
        leftPPT.setControl(
            leftPPTVoltageRequest
                .withOutput(speed.voltage())
        );
    }

    public void setIndexerSpeed(IndexerSpeed speed){
        indexer.setVoltage(speed.voltage());
    }

    // ---------------------------------------------------

    // COMMANDS ------------------------------------------
    public Command deliverCommand() {
        return startEnd(
            () -> {
                setIndexerSpeed(IndexerSpeed.FORWARD);
                setPPTSpeed(PPTSpeed.FORWARD);
            },
            () -> {
                setIndexerSpeed(IndexerSpeed.STOP);
                setPPTSpeed(PPTSpeed.STOP);
            }
        );
    }

    public Command reverseDeliverCommand() {
        return startEnd(
            () -> {
                setIndexerSpeed(IndexerSpeed.REVERSE);
                setPPTSpeed(PPTSpeed.REVERSE);
            },
            () ->  {
                setIndexerSpeed(IndexerSpeed.STOP);
                setPPTSpeed(PPTSpeed.STOP);
            }
        );
    }

    //--------------------------------------------------------




    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

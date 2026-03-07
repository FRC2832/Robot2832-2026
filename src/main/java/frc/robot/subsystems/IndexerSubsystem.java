// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    /** Creates a new HopperSubsystem. */

    // ENUMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -------------------------
    public enum Speed {
        STOP(0),
        FORWARD(.8),
        REVERSE(-.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    // -------------------------------------------------------

    private SparkFlex indexer;

    public IndexerSubsystem() {
        indexer = new SparkFlex(Constants.INDEXER_ID, Constants.INDEXER_MOTOR_TYPE); // I don't configure this motor.
    }


    // Speed controls ----------------------------------
    public void setIndexerSpeed(Speed speed){
        indexer.setVoltage(speed.voltage());
    }

    // ---------------------------------------------------
    // COMMANDS ------------------------------------------
    
    // TODO: This should be made the default command but since we're only doing subsystem setup right now I didn't do that.
    public Command deliverCommand() {
        return startEnd(
            () -> {
                setIndexerSpeed(Speed.FORWARD);
            },
            () -> {
                setIndexerSpeed(Speed.STOP);
            }
        );
    }

    public Command reverseDeliverCommand() {
        return startEnd(
            () -> {
                setIndexerSpeed(Speed.REVERSE);
            },
            () ->  {
                setIndexerSpeed(Speed.STOP);
            }
        );
    }

    //--------------------------------------------------------



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

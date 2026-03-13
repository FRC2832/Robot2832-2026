// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    /** Creates a new HopperSubsystem. */

    // ENUMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -------------------------
    public enum Speed {
        STOP(0),
        FORWARD(0.1),
        REVERSE(-0.1);

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
        indexer.configure(new SparkFlexConfig().inverted(true), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putData(this); 
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
            () -> setIndexerSpeed(Speed.FORWARD),
            () -> setIndexerSpeed(Speed.STOP)
        );
    }

    public Command reverseDeliverCommand() {
        return startEnd(
            () -> setIndexerSpeed(Speed.REVERSE),
            () -> setIndexerSpeed(Speed.STOP)
        );
    }

    //--------------------------------------------------------



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTurretCommand extends Command {
    /** Creates a new MoveTurretCommand. */

    public MoveTurretCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.turretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(TurretSubsystem.autoAim){
            //TODO add auto aim controls
            RobotContainer.turretSubsystem.setTurretAngle(0, 0);
        }else{
            double joystick = RobotContainer.operatorController.getLeftX();
            RobotContainer.turretSubsystem.setTurretVoltage(joystick * 4, joystick * 4);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

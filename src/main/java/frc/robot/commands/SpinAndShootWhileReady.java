// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

import frc.robot.subsystems.PPTSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinAndShootWhileReady extends Command {

    //public static double TARGET_SPEED = 54;
    int cyclesSinceLastPush = Integer.MAX_VALUE;
    private static final int CYCLES_TO_CONTINUE = 0;

    /** Creates a new ShootWhileReady. */
    public SpinAndShootWhileReady() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.shooterSubsystem, RobotContainer.pptSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double leftSpeed = SpinShooterCommand.leftSpeed.getAsDouble();
        double rightSpeed = SpinShooterCommand.rightSpeed.getAsDouble();
        double accelSpeed = SpinShooterCommand.acceleratorSpeed.getAsDouble();
        RobotContainer.shooterSubsystem.setMotorSpeed(rightSpeed, leftSpeed, accelSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftSpeed = SpinShooterCommand.leftSpeed.getAsDouble();
        double rightSpeed = SpinShooterCommand.rightSpeed.getAsDouble();
        double accelSpeed = SpinShooterCommand.acceleratorSpeed.getAsDouble();
        double leftTargetSpeed = leftSpeed*100-1;
        double rightTargetSpeed = rightSpeed*100-1;
        RobotContainer.shooterSubsystem.setMotorSpeed(rightSpeed, leftSpeed, accelSpeed);
        if (RobotContainer.shooterSubsystem.getLeftMotorSpeed() > leftTargetSpeed
                && RobotContainer.shooterSubsystem.getRightMotorSpeed() > rightTargetSpeed) {
            RobotContainer.pptSubsystem.setPPTSpeed(PPTSubsystem.Speed.FORWARD, PPTSubsystem.Speed.FORWARD);
            cyclesSinceLastPush = 0;
        } else if (RobotContainer.shooterSubsystem.getLeftMotorSpeed() > leftTargetSpeed) {
            RobotContainer.pptSubsystem.setPPTSpeed(PPTSubsystem.Speed.STOP, PPTSubsystem.Speed.FORWARD);
            cyclesSinceLastPush = 0;
        } else if (RobotContainer.shooterSubsystem.getRightMotorSpeed() > rightTargetSpeed) {
            RobotContainer.pptSubsystem.setPPTSpeed(PPTSubsystem.Speed.FORWARD, PPTSubsystem.Speed.STOP);
            cyclesSinceLastPush = 0;
        } else if (cyclesSinceLastPush < CYCLES_TO_CONTINUE) {
            cyclesSinceLastPush++;
        } else {
            RobotContainer.pptSubsystem.setPPTSpeed(PPTSubsystem.Speed.STOP, PPTSubsystem.Speed.STOP);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooterSubsystem.setMotorSpeed(0, 0, 0);
        RobotContainer.pptSubsystem.setPPTSpeed(PPTSubsystem.Speed.STOP, PPTSubsystem.Speed.STOP);
        cyclesSinceLastPush = Integer.MAX_VALUE;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinShooterCommand extends Command {
    /** Creates a new ShootCommand. */
    static DoubleSupplier leftSpeed, rightSpeed, acceleratorSpeed;
   
    public SpinShooterCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DoubleSupplier acceleratorSpeed) {
        addRequirements(RobotContainer.shooterSubsystem);
        setSuppliers(leftSpeed, rightSpeed, acceleratorSpeed);
    }

    //TODO set to use automatic suppliers
    public SpinShooterCommand(){
        this(ShooterSubsystem::getLastLeftSpeed, ShooterSubsystem::getLastRightSpeed, ShooterSubsystem::getLastAcceleratorSpeed);
    }

    public static void setSuppliers(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DoubleSupplier acceleratorSpeed){
        SpinShooterCommand.leftSpeed = leftSpeed;
        SpinShooterCommand.rightSpeed = rightSpeed;
        SpinShooterCommand.acceleratorSpeed = acceleratorSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ShooterSubsystem.leftSpeed = leftSpeed.getAsDouble();
        ShooterSubsystem.rightSpeed = rightSpeed.getAsDouble();
        ShooterSubsystem.acceleratorSpeed = acceleratorSpeed.getAsDouble();
        RobotContainer.shooterSubsystem.setMotorSpeed(ShooterSubsystem.rightSpeed, ShooterSubsystem.leftSpeed, ShooterSubsystem.acceleratorSpeed);
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

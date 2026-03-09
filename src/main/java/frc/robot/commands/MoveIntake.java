// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveIntake extends Command {
    /** Creates a new LowerIntake. */
    private static final Timer timer = new Timer();
    boolean isDown;
    double voltage;
    public MoveIntake(boolean down) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.intakeSubsystem);
        this.isDown = down;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double voltage = Constants.INTAKE_EXTEND_VOLTAGE;
        if(isDown)
            voltage *= -1;
        this.voltage = voltage;
        RobotContainer.intakeSubsystem.intakeExtenderMotor.setVoltage(voltage);
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.intakeSubsystem.intakeExtenderMotor.setVoltage(this.voltage);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.intakeSubsystem.intakeExtenderMotor.setVoltage(0);
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //Supply current increases when the resistance increases
        return timer.get() > 4 || Math.abs(RobotContainer.intakeSubsystem.intakeExtenderMotor.getSupplyCurrent().getValue().in(Amps)) > 12;
    }
}

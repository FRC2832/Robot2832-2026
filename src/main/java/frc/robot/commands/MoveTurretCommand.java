// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.TurretNoYams;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTurretCommand extends Command {
    /** Creates a new MoveTurretCommand. */

    TurretNoYams turret;
    static boolean isOnRed;
    boolean isLeft;
    DoubleSupplier joystickX, joystickY;

    public MoveTurretCommand(TurretNoYams turret) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
        this.turret = turret;
        isLeft = turret.isLeftTurret();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isOnRed = Utils.isOnRed();
        joystickX = getJoystickX();
        joystickY = getJoystickY();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xPos = joystickX.getAsDouble();
        if (Math.abs(xPos) > Constants.TURRET_CONTROL_DEADZONE) {
            turret.isAutoAim = false;
            if(isLeft)
                RobotContainer.logger.leftTurretAutoAiming.set(false);
            else
                RobotContainer.logger.rightTurretAutoAiming.set(false);
            //TODO decide if we should turn off autoaim for all turrets or just this one
        }
        if(Math.abs(joystickY.getAsDouble()) > Constants.TURRET_CONTROL_DEADZONE){
            if(isLeft)
                RobotContainer.shooterSubsystem.isLeftAutoAim = false;
            else
                RobotContainer.shooterSubsystem.isRightAutoAim = false;
        }
        if (turret.isAutoAim) {
            turret.aimAtPosition(Utils.getTargetPosition(), RobotContainer.drivetrain.getPose());
        } else {
            turret.setVoltage(Volts.of(1.5 * MathUtil.applyDeadband(xPos, Constants.TURRET_CONTROL_DEADZONE)));
        }
    }

    private DoubleSupplier getJoystickX() {
        if (isLeft) {
            return () -> RobotContainer.operatorController.getLeftX();
        }
        return () -> RobotContainer.operatorController.getRightX();
    }

    private DoubleSupplier getJoystickY() {
        if (isLeft) {
            return () -> RobotContainer.operatorController.getLeftY();
        }
        return () -> RobotContainer.operatorController.getRightY();
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

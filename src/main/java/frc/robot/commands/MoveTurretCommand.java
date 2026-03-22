// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTurretCommand extends Command {
    /** Creates a new MoveTurretCommand. */

    TurretSubsystem turret;
    static boolean isOnRed;
    boolean isLeft;
    DoubleSupplier joystickX, joystickY;

    public MoveTurretCommand(TurretSubsystem turret) {
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
        if (Math.abs(xPos) > 0.1 || Math.abs(joystickY.getAsDouble()) > 0.1)
            turret.isAutoAim = false;
        if (turret.isAutoAim) {
            turret.aimAtPosition(MoveTurretCommand.getTargetPosition(), RobotContainer.drivetrain.getPose());
        } else {
            turret.setVoltage(Volts.of(6 * MathUtil.applyDeadband(xPos, 0.1)));
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

    private static Translation2d getTargetPosition() {
        Pose2d robotPose = RobotContainer.drivetrain.getPose();

        if (Utils.inAllianceZone(robotPose)) {
            if (isOnRed) {
                return Constants.RED_HUB_POS;
            } else {
                return Constants.BLUE_HUB_POS;
            }
        } else {
            if (isOnRed) {
                if (robotPose.getMeasureY().gt(Constants.RED_HUB_POS.getMeasureY())) {
                    return Constants.RED_RIGHT_SNOWBLOW_TARGET;
                } else {
                    return Constants.RED_LEFT_SNOWBLOW_TARGET;
                }
            } else {
                if (robotPose.getMeasureY().gt(Constants.BLUE_HUB_POS.getMeasureY())) {
                    return Constants.BLUE_LEFT_SNOWBLOW_TARGET;
                } else {
                    return Constants.BLUE_RIGHT_SNOWBLOW_TARGET;
                }
            }
        }
    }

}

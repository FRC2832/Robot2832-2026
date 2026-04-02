// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.util.LookupTable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveHoodCommand extends Command {
    /** Creates a new MoveHoodCommand. */
    HoodSubsystem hood;
    boolean isLeftHood;
    DoubleSupplier joystickY;

    public MoveHoodCommand(HoodSubsystem hood) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(hood);
        this.hood = hood;
        isLeftHood = hood.isLeftTurretHood();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // System.out.println("Moving hood by " +
        // (-RobotContainer.operatorController.getRightY()));
        joystickY = getJoystickY();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (hood.isAutoAim()) {
            Translation2d target = Utils.getTargetPosition();
            Translation2d robotPos = RobotContainer.drivetrain.getPose().getTranslation();
            Distance dist = Meters.of(target.getDistance(robotPos));
            LookupTable.Result lookupResult = Constants.SHOOTER_LOOKUP_TABLE.lookup(dist);
            hood.setHoodPosition(lookupResult.hoodServoSetting());
        } else {
            double joystick = MathUtil.applyDeadband(joystickY.getAsDouble(), Constants.TURRET_CONTROL_DEADZONE);
            hood.offsetHood(-joystick);
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

    private DoubleSupplier getJoystickY() {
        if (isLeftHood) {
            return () -> RobotContainer.operatorController.getLeftY();
        }
        return () -> RobotContainer.operatorController.getRightY();
    }
}

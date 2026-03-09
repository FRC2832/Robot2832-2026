// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.util.StatusLogger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PPTSubsystem;
import frc.robot.commands.DriveSpeedCMDs;
import frc.robot.commands.MoveHoodCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = 0.8 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                            // second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static CommandXboxController driverController;
    public static CommandXboxController operatorController;

    public static CommandSwerveDrivetrain drivetrain;

    // Variables added in non-swerve project creation
    private double speedMultiplier = 1.0;

    // Subsytem declaration
    public static ShooterSubsystem shooterSubsystem;
    public static TurretSubsystem turretSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static IndexerSubsystem indexerSubsystem;
    public static PPTSubsystem pptSubsystem;
    public static HoodSubsystem hoodSubsystem;

    private static final double shooterManualStepSize = 0.05;

    public RobotContainer() {
        // PathPlannerLogging.clearLoggingCallbacks();
        // SignalLogger.enableAutoLogging(false);
        // StatusLogger.disableAutoLogging();
        drivetrain = TunerConstants.createDrivetrain();
        shooterSubsystem = new ShooterSubsystem();
        turretSubsystem = new TurretSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        indexerSubsystem = new IndexerSubsystem();
        pptSubsystem = new PPTSubsystem();
        hoodSubsystem = new HoodSubsystem();
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        configureBindings();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * speedMultiplier) // Drive
                                                                                                             // forward
                                                                                                             // with
                                // negative Y
                                // (forward)
                                .withVelocityY(-driverController.getLeftX() * MaxSpeed * speedMultiplier) // Drive left
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate * speedMultiplier) // Drive
                                                                                                                      // counterclockwise
                                                                                                                      // with
                // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // TODO: Test if this works to actually reset field orientation
        // Reset the field-centric heading on start press.
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Operator controls
        operatorController.rightBumper().whileTrue(pptSubsystem.deliverCommand());
        operatorController.leftBumper().whileTrue(pptSubsystem.reverseDeliverCommand());

        operatorController.rightTrigger(.3).whileTrue(intakeSubsystem.runIntakeCommand());
        operatorController.leftTrigger(.3).whileTrue(intakeSubsystem.reverseIntakeCommand());
        // operatorController.start().onTrue(intakeSubsystem.extendIntakeCommand());
        // operatorController.back().onTrue(intakeSubsystem.retractIntakeCommand());

        // Default Commands
        indexerSubsystem.setDefaultCommand(indexerSubsystem.deliverCommand());

        // Speed control keys (Variable speed control)
        driverController.leftTrigger(0.15)
                .whileTrue(new DriveSpeedCMDs(this, 1.0 - driverController.getLeftTriggerAxis(), driverController));
        // driverController.leftTrigger(0.2).whileTrue(new DriveSpeedCMDs(this, 0.7));
        // // Turtle
        // driverController.leftTrigger(0.6).whileTrue(new DriveSpeedCMDs(this, 0.4));
        // // Snail
        driverController.y().toggleOnTrue(RobotContainer.drivetrain.applyRequest(() -> brake));

        shooterSubsystem.setDefaultCommand(new SpinShooterCommand());
        // Shooter manual speed adjustment
        operatorController.povUp().onTrue(shooterSubsystem.runOnce(() -> {
            SpinShooterCommand.setSuppliers(ShooterSubsystem::getLastLeftSpeed, ShooterSubsystem::getLastRightSpeed,
                    ShooterSubsystem::getLastAcceleratorSpeed);
            ShooterSubsystem.leftSpeed += shooterManualStepSize;
            ShooterSubsystem.rightSpeed += shooterManualStepSize;
            ShooterSubsystem.acceleratorSpeed += shooterManualStepSize * .4;
        }));
        operatorController.povDown().onTrue(shooterSubsystem.runOnce(() -> {
            SpinShooterCommand.setSuppliers(ShooterSubsystem::getLastLeftSpeed, ShooterSubsystem::getLastRightSpeed,
                    ShooterSubsystem::getLastAcceleratorSpeed);
            ShooterSubsystem.leftSpeed -= shooterManualStepSize;
            ShooterSubsystem.rightSpeed -= shooterManualStepSize;
            ShooterSubsystem.acceleratorSpeed -= shooterManualStepSize *.4;
        }));
        // TODO add switch back to automatic targeting
        // On operator Y press, call ShootCommand.setSuppliers with the automatic
        // targeting suppliers
        new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.1).whileTrue(new MoveHoodCommand());
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }

    // Used in DriveSpeedCMDs to set speedMultiplier
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }
}

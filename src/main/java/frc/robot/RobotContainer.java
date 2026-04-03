// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeExtenderSubsystem;
import frc.robot.subsystems.PPTSubsystem;
import frc.robot.commands.DriveSpeedCMDs;
import frc.robot.commands.MoveHoodCommand;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveTurretCommand;
import frc.robot.commands.SpinAndShootWhileReady;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretNoYams;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private static double MaxSpeed = 0.8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                               // desired top
                                                                                               // speed
    private double MaxAngularRate = 0.9 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                            // second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // For use with pathplanner
    public static final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final Telemetry logger = new Telemetry(MaxSpeed);

    public static CommandXboxController driverController;
    public static CommandXboxController operatorController;

    public static CommandSwerveDrivetrain drivetrain;

    // Variables added in non-swerve project creation
    private double speedMultiplier = 1.0;

    // Subsytem declaration
    public static ShooterSubsystem shooterSubsystem;
    // public static TurretSubsystem_Old turretSubsystem;
    public static TurretNoYams leftTurretSubsystem;
    public static TurretNoYams rightTurretSubsystem;
    public static IntakeRollerSubsystem intakeRollerSubsystem;
    public static IntakeExtenderSubsystem intakeExtenderSubsystem;
    public static IndexerSubsystem indexerSubsystem;
    public static PPTSubsystem pptSubsystem;
    public static HoodSubsystem leftHoodSubsystem;
    public static HoodSubsystem rightHoodSubsystem;
    public static VisionSubsystem visionSubsystem;

    private static final double shooterManualStepSize = 0.05;

    SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // PathPlannerLogging.clearLoggingCallbacks();
        // SignalLogger.enableAutoLogging(false);
        // StatusLogger.disableAutoLogging();
        drivetrain = TunerConstants.createDrivetrain();
        shooterSubsystem = new ShooterSubsystem();
        leftTurretSubsystem = new TurretNoYams(true, InvertedValue.CounterClockwise_Positive,
                SensorDirectionValue.Clockwise_Positive, Constants.LEFT_TURRET_ENCODER_OFFSET);
        rightTurretSubsystem = new TurretNoYams(false, InvertedValue.CounterClockwise_Positive,
                SensorDirectionValue.Clockwise_Positive, Constants.RIGHT_TURRET_ENCODER_OFFSET);
        intakeRollerSubsystem = new IntakeRollerSubsystem();
        intakeExtenderSubsystem = new IntakeExtenderSubsystem();
        indexerSubsystem = new IndexerSubsystem();
        pptSubsystem = new PPTSubsystem();
        leftHoodSubsystem = new HoodSubsystem(leftTurretSubsystem);
        rightHoodSubsystem = new HoodSubsystem(rightTurretSubsystem);
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        visionSubsystem = new VisionSubsystem();
        configureBindings();
        // Auto Chooser Setup
        // -----------------------------------------------------------------------------------------------------------------------------
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                stream -> stream.filter(auto -> auto.getName().startsWith("Woodhaven")));
        // autoChooser.setDefaultOption("Hub Shoot Once", new PathPlannerAuto("Hub Shoot
        // Once"));
        // autoChooser.addOption("Left Bump Shoot Once", new PathPlannerAuto("Woodhaven
        // Left Bump Shoot Once"));
        // autoChooser.addOption("Right Bump Shoot Once", new PathPlannerAuto("Woodhaven
        // Right Bump Shoot Once"));
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    private void configureBindings() {

        // Default Commands
        // -------------------------------------------------------------------------------------------------------------------------------
        indexerSubsystem.setDefaultCommand(indexerSubsystem.deliverCommand());
        shooterSubsystem.setDefaultCommand(shooterSubsystem.stopShooter());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed
                                * speedMultiplier) // Drive forward with negative Y (forward)
                                .withVelocityY(-driverController.getLeftX() * MaxSpeed
                                        * speedMultiplier) // Drive left with negative X (left)
                                .withRotationalRate(-driverController.getRightX()
                                        * MaxAngularRate * speedMultiplier) // Drive counterclockwise with negative X
                                                                            // (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(),
                                -driverController.getLeftX()))));
        driverController.x().whileTrue(drivetrain.aimTowardsHub());
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // TODO: Test if this works to actually reset field orientation
        // Reset the field-centric heading on start press.
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Speed control keys (Variable speed control)
        driverController.leftTrigger(0.15)
                .whileTrue(new DriveSpeedCMDs(this, 1.0 - driverController.getLeftTriggerAxis(),
                        driverController));
        // driverController.leftTrigger(0.2).whileTrue(new DriveSpeedCMDs(this, 0.7));
        // // Turtle
        // driverController.leftTrigger(0.6).whileTrue(new DriveSpeedCMDs(this, 0.4));
        // // Snail
        driverController.y().toggleOnTrue(RobotContainer.drivetrain.applyRequest(() -> brake));

        // Operator controls
        // -------------------------------------------------------------------------------------------------------------------------------
        // Intake ************************
        operatorController.back().whileTrue(new MoveIntake(false));
        operatorController.start().whileTrue(new MoveIntake(true));

        operatorController.rightTrigger(.3).whileTrue(intakeRollerSubsystem.runIntakeCommand());
        operatorController.leftTrigger(.3).whileTrue(intakeRollerSubsystem.reverseIntakeCommand());

        // PPT / Delivering ***************
        operatorController.rightBumper()
                .whileTrue(pptSubsystem.deliverCommand().alongWith(new SpinShooterCommand()));
        operatorController.leftBumper().whileTrue(pptSubsystem.reverseDeliverCommand()
                .alongWith(indexerSubsystem.reverseDeliverCommand()
                        .alongWith(shooterSubsystem.reverseShooter())));

        // Shooter ************************
        //Move shooter up and down while shooting and not intaking
        // operatorController.y().or(operatorController.a())
        //     .and(operatorController.rightTrigger(.1).negate())
        //     .and(operatorController.rightTrigger(.1).negate())
        //     .whileTrue(
        //         intakeExtenderSubsystem.extendIntakeCommand()
        //         .andThen(new WaitCommand(1))
        //         .andThen(intakeExtenderSubsystem.retractIntakeCommand())
        //         .andThen(new WaitCommand(1))
        //     ).onFalse(intakeExtenderSubsystem.extendIntakeCommand());

        operatorController.y().whileTrue(new SpinShooterCommand());
        operatorController.a().whileTrue(new SpinAndShootWhileReady());

        operatorController.b().onTrue(toggleAutoAim());
        leftTurretSubsystem.setDefaultCommand(new MoveTurretCommand(leftTurretSubsystem));
        rightTurretSubsystem.setDefaultCommand(new MoveTurretCommand(rightTurretSubsystem));

        // Shooter manual speed adjustment
        operatorController.povUp().onTrue(shooterSubsystem.runOnce(() -> {
            SpinShooterCommand.setSuppliers(ShooterSubsystem::getLastLeftSpeedProportion,
                    ShooterSubsystem::getLastRightSpeedProportion,
                    ShooterSubsystem::getLastAcceleratorSpeedProportion);
            ShooterSubsystem.leftSpeed += shooterManualStepSize;
            ShooterSubsystem.rightSpeed += shooterManualStepSize;
            disableAutoAim();
            // ShooterSubsystem.acceleratorSpeed += shooterManualStepSize;
            // SpinAndShootWhileReady.TARGET_SPEED += 5;
            // System.out.println("Shooters: " + ShooterSubsystem.leftSpeed +
            // "\nAccelerator" + ShooterSubsystem.acceleratorSpeed);
        }));
        operatorController.povDown().onTrue(shooterSubsystem.runOnce(() -> {
            SpinShooterCommand.setSuppliers(ShooterSubsystem::getLastLeftSpeedProportion,
                    ShooterSubsystem::getLastRightSpeedProportion,
                    ShooterSubsystem::getLastAcceleratorSpeedProportion);
            ShooterSubsystem.leftSpeed -= shooterManualStepSize;
            ShooterSubsystem.rightSpeed -= shooterManualStepSize;
            disableAutoAim();
            // ShooterSubsystem.acceleratorSpeed -= shooterManualStepSize;
            // SpinAndShootWhileReady.TARGET_SPEED -= 5;
            // System.out.println("Shooters: " + ShooterSubsystem.leftSpeed +
            // "\nAccelerator" + ShooterSubsystem.acceleratorSpeed);
        }));
        // TODO add switch back to automatic targeting
        // On operator Y press, call ShootCommand.setSuppliers with the automatic
        // targeting suppliers
        leftHoodSubsystem.setDefaultCommand(new MoveHoodCommand(leftHoodSubsystem));
        rightHoodSubsystem.setDefaultCommand(new MoveHoodCommand(rightHoodSubsystem));
        // Commands for auton
        NamedCommands.registerCommand("Shoot", new SpinAndShootWhileReady().withTimeout(3.5));
        NamedCommands.registerCommand("Reverse PPT", pptSubsystem.reverseDeliverCommand().withTimeout(3));
        NamedCommands.registerCommand("Extend Ingestor", intakeExtenderSubsystem.extendIntakeCommand());
        NamedCommands.registerCommand("Start Ingestor", intakeRollerSubsystem.startIntakeCommand());
        NamedCommands.registerCommand("Stop Ingestor", intakeRollerSubsystem.stopIntakeCommand());
        NamedCommands.registerCommand("Raise Ingestor", intakeExtenderSubsystem.retractIntakeCommand());

        SmartDashboard.putData("Test", rightTurretSubsystem.runOnce(
                () -> rightTurretSubsystem.setAngle(Degrees.of(30))));
    }

    public static void enableAutoAim(){
        leftTurretSubsystem.isAutoAim = false; //TODO fix turret aiming and reenable
        rightTurretSubsystem.isAutoAim = false;
        shooterSubsystem.isLeftAutoAim = true;
        shooterSubsystem.isRightAutoAim = true;
        logger.leftTurretAutoAiming.set(true);
        logger.rightTurretAutoAiming.set(true);
        SpinShooterCommand.setToAutoSuppliers();
    }

    public static void disableAutoAim(){
        leftTurretSubsystem.isAutoAim = false;
        rightTurretSubsystem.isAutoAim = false;
        shooterSubsystem.isLeftAutoAim = false;
        shooterSubsystem.isRightAutoAim = false;
        logger.leftTurretAutoAiming.set(false);
        logger.rightTurretAutoAiming.set(false);
        SpinShooterCommand.setToManualSuppliers();
    }

    public Command toggleAutoAim() {
        return Commands.runOnce(() -> {
            if (leftTurretSubsystem.isAutoAim || rightTurretSubsystem.isAutoAim) {
                enableAutoAim();
            } else {
                disableAutoAim();
            }
        });
    }

    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();
        if(auto instanceof PathPlannerAuto ppAuto){
            Pose2d start = ppAuto.getStartingPose();
            if (Utils.isOnRed())
                start = FlippingUtil.flipFieldPose(start);
            drivetrain.resetPose(start);
        }else if(auto instanceof InstantCommand instant){
            String name = instant.getName();
            if(new File(Filesystem.getDeployDirectory(), 
                    "pathplanner/autos" + name + ".auto").exists()){
                Pose2d start = new PathPlannerAuto(name).getStartingPose();
                if (Utils.isOnRed())
                    start = FlippingUtil.flipFieldPose(start);
                drivetrain.resetPose(start);
            }
        }
        //return startAutoAim().alongWith(autoChooser.getSelected());
        return auto;
        // return new PathPlannerAuto("Hub Shoot Once");
        // // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        // // Reset our field centric heading to match the robot
        // // facing away from our alliance station wall (0 deg).
        // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // // Then slowly drive forward (away from us) for 5 seconds.
        // drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
        // .withVelocityY(0)
        // .withRotationalRate(0))
        // .withTimeout(5.0),
        // // Finally idle for the rest of auton
        // drivetrain.applyRequest(() -> idle));
    }

    // Used in DriveSpeedCMDs to set speedMultiplier
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }

}

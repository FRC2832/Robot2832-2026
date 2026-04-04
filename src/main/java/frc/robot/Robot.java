// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.PhaseChecker;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    // matchTime will always count DOWN. Because that's how drive station gives it to you.
    private double matchTime = -1;
    private boolean hubActive = false;
    private static final double[] SHIFT_TIMES = {130, 105, 80, 55, 30};
    private int nextPhaseIndex = 0;


    Elastic.Notification hubActiveSoonNotification = new Elastic.Notification(Elastic.NotificationLevel.INFO,
                                                                        "HUB ACTIVATING!!!!!!!!!!",
                                                                  "GO HOOOOOMEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE",
                                                            2000,
                                                                        500,
                                                                        500);
    Elastic.Notification hubDeactivatingSoonNotification = new Elastic.Notification(Elastic.NotificationLevel.ERROR, // I used error cuz it's red but that might be scary.
                                                                        "hub is going bye bye :(",
                                                                  "hurry up!",
                                                            2000,
                                                                        500,
                                                                        500);

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        Elastic.selectTab("Autonomous"); // I heard a rumor that if you don't run an elastic command in init the robot explodes?
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        // FIX: this stuff (+ other stuff possibly?) makes the loops take like 0.002 seconds which is far too long but I'm not fixing it tonight. Hashtag plx fix @kransaw.

        matchTime = DriverStation.getMatchTime();
        hubActive = PhaseChecker.isHubActive(matchTime);

        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putBoolean("Hub Active", hubActive);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        Utils.recheckTeam();
        RobotContainer.leftTurretSubsystem.isAutoAim = Constants.SHOULD_AUTO_AIM_TURRET_AT_START;
        RobotContainer.rightTurretSubsystem.isAutoAim = Constants.SHOULD_AUTO_AIM_TURRET_AT_START;
        RobotContainer.shooterSubsystem.isLeftAutoAim = Constants.SHOULD_AUTO_SET_SPEED_AT_START;
        RobotContainer.shooterSubsystem.isRightAutoAim = Constants.SHOULD_AUTO_SET_SPEED_AT_START;
        RobotContainer.logger.leftTurretAutoAiming.set(Constants.SHOULD_AUTO_AIM_TURRET_AT_START);
        RobotContainer.logger.rightTurretAutoAiming.set(Constants.SHOULD_AUTO_AIM_TURRET_AT_START);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        Elastic.selectTab("Autonomous");
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        Utils.recheckTeam();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        nextPhaseIndex = 0;

        Elastic.selectTab("Teleop");
    }

    @Override
    public void teleopPeriodic() {
        // Checks if we're 1. in bounds 2. near a phase shift and 3. The hub will be active during that shift. This is an insane way to write this code.
        // Then waits till the next phase by iterating upon next phase index.
        // Also checks if the match time is above -1 because otherwise you get one million notifications when just running teleop.
        if(matchTime > -1){
            if(nextPhaseIndex < SHIFT_TIMES.length && matchTime <= (SHIFT_TIMES[nextPhaseIndex] + 3) && PhaseChecker.isHubActive(matchTime - 4)){
                Elastic.sendNotification(hubActiveSoonNotification);
                nextPhaseIndex++;
            }
            // This does the same thing but for an inactive hub.
            else if(nextPhaseIndex < SHIFT_TIMES.length && matchTime <= (SHIFT_TIMES[nextPhaseIndex] + 3) && !PhaseChecker.isHubActive(matchTime - 4)){
                Elastic.sendNotification(hubDeactivatingSoonNotification);
                nextPhaseIndex++;
            }
        }
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}

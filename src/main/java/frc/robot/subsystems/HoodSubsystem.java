// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class HoodSubsystem extends SubsystemBase {
    private static ServoHub servoHub = null;
    private ServoChannel leftServo, rightServo;
    private double hoodPosition = 0;
    private boolean isLeftTurret;
    private BooleanSupplier isAutoAim;

    /** Creates a new HoodSubsystem. */
    public HoodSubsystem(TurretSubsystem turret) {
        this.isLeftTurret = turret.isLeftTurret();
        isAutoAim = () -> turret.isAutoAim;
        if (servoHub == null) {
            servoHub = new ServoHub(Constants.SERVO_HUB_ID);
            ServoHubConfig config = new ServoHubConfig();
            config.channel0.pulseRange(1000, 1500, 2000);
            config.channel1.pulseRange(1000, 1500, 2000);
            config.channel4.pulseRange(1000, 1500, 2000);
            config.channel5.pulseRange(1000, 1500, 2000);
            servoHub.configure(config, ResetMode.kResetSafeParameters);
        }
        if (isLeftTurret) {
            leftServo = servoHub.getServoChannel(ChannelId.kChannelId0);
            rightServo = servoHub.getServoChannel(ChannelId.kChannelId1);
        } else {
            leftServo = servoHub.getServoChannel(ChannelId.kChannelId5);
            rightServo = servoHub.getServoChannel(ChannelId.kChannelId4);
        }
        configureServo(leftServo);
        configureServo(rightServo);
        setHoodPosition(0);
        SmartDashboard.putData(this);
    }

    private void configureServo(ServoChannel channel) {
        if(isHoodEnabled()){
            channel.setEnabled(true);
            channel.setPowered(true);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setHoodPosition(double hoodPosition) {
        if(isLeftTurret)
            RobotContainer.logger.leftHoodValue.set(hoodPosition);
        else
            RobotContainer.logger.rightHoodValue.set(hoodPosition);
        if(isHoodEnabled()){
            setServo(leftServo, MathUtil.clamp(hoodPosition, -1, 1));
            setServo(rightServo, MathUtil.clamp(hoodPosition, -1, 1));
        }
        // setServo(rightHoodLeftServo, 0.6 * MathUtil.clamp(rightHoodPosition, -1, 1));
        // setServo(rightHoodRightServo, 0.6 * MathUtil.clamp(rightHoodPosition, -1,
        // 1));
    }

    private void setServo(ServoChannel servo, double position) {
        int pulseWidth = (int) (1500 + 500 * position);
        servo.setPulseWidth(pulseWidth);
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public void offsetHood(double hoodOffset) {
        hoodPosition += Constants.HOOD_SENSITIVITY * hoodOffset;
        hoodPosition = MathUtil.clamp(hoodPosition, -1, 0);
        setHoodPosition(hoodPosition);
    }

    public boolean isAutoAim() {
        return isAutoAim.getAsBoolean();
    }

    public boolean isLeftTurretHood() {
        return isLeftTurret;
    }

    private boolean isHoodEnabled(){
        return isLeftTurret ? Constants.LEFT_HOOD_ENABLED : Constants.RIGHT_HOOD_ENABLED;
    }
}

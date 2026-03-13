// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private ServoHub servoHub;
    private ServoChannel leftHoodLeftServo, leftHoodRightServo, rightHoodLeftServo, rightHoodRightServo;
    private double leftHoodPosition, rightHoodPosition;

    /** Creates a new HoodSubsystem. */
    public HoodSubsystem() {
        servoHub = new ServoHub(Constants.SERVO_HUB_ID);
        ServoHubConfig config = new ServoHubConfig();
        config.channel0.pulseRange(1000, 1500, 2000);
        config.channel1.pulseRange(1000, 1500, 2000);
        config.channel4.pulseRange(1000, 1500, 2000);
        config.channel5.pulseRange(1000, 1500, 2000);
        servoHub.configure(config, ResetMode.kResetSafeParameters);
        leftHoodLeftServo = servoHub.getServoChannel(ChannelId.kChannelId0);
        leftHoodRightServo = servoHub.getServoChannel(ChannelId.kChannelId1);
        rightHoodLeftServo = servoHub.getServoChannel(ChannelId.kChannelId5);
        rightHoodRightServo = servoHub.getServoChannel(ChannelId.kChannelId4);
        configureServo(leftHoodLeftServo);
        configureServo(leftHoodRightServo);
        configureServo(rightHoodLeftServo);
        configureServo(rightHoodRightServo);

        SmartDashboard.putData(this); 
    }

    private void configureServo(ServoChannel channel) {
        channel.setEnabled(true);
        channel.setPowered(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setHoodPositions(double leftHoodPosition, double rightHoodPosition) {
        this.leftHoodPosition = leftHoodPosition;
        this.rightHoodPosition = rightHoodPosition;
        setServo(leftHoodLeftServo, 0.3 * MathUtil.clamp(leftHoodPosition, -1, 1));
        setServo(leftHoodRightServo, 0.3 * MathUtil.clamp(leftHoodPosition, -1, 1));
        setServo(rightHoodLeftServo, 0.3 * MathUtil.clamp(rightHoodPosition, -1, 1));
        setServo(rightHoodRightServo, 0.3 * MathUtil.clamp(rightHoodPosition, -1, 1));
    }

    private void setServo(ServoChannel servo, double position) {
        int pulseWidth = (int) (1500 + 500 * position);
        servo.setPulseWidth(pulseWidth);
    }

    public double getLeftHoodPosition() {
        return leftHoodPosition;
    }

    public double getRightHoodPosition() {
        return rightHoodPosition;
    }

    public void setHoodSpeeds(double leftHoodSpeed, double rightHoodSpeed) {
        leftHoodPosition += Constants.HOOD_SENSITIVITY * leftHoodSpeed;
        rightHoodPosition += Constants.HOOD_SENSITIVITY * rightHoodSpeed;
        leftHoodPosition = MathUtil.clamp(leftHoodPosition, -1, 1);
        rightHoodPosition = MathUtil.clamp(rightHoodPosition, -1, 1);
        // System.out.println("Hood target positions: " + leftHoodPosition + ", " +
        // rightHoodPosition);
        setHoodPositions(leftHoodPosition, rightHoodPosition);
    }

}

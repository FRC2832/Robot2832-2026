// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

  public PhotonCamera leftCam;
  PhotonPoseEstimator leftPoseEstimator;
  PhotonCameraSim leftCamSim;

  public PhotonCamera rightCam;
  PhotonPoseEstimator rightPoseEstimator;
  PhotonCameraSim rightCamSim;
  
  VisionSystemSim visionSim;

  public List<PhotonPipelineResult> leftResults, rightResults;

  public VisionSubsystem() {
    super();
    leftCam = new PhotonCamera(Constants.leftCamName);
    leftPoseEstimator = new PhotonPoseEstimator(Constants.tagLayout, Constants.leftCamPose);
    rightCam = new PhotonCamera(Constants.rightCamName);
    rightPoseEstimator = new PhotonPoseEstimator(Constants.tagLayout, Constants.rightCamPose);
    leftResults = new LinkedList<>();
    rightResults = new LinkedList<>();
    if(Robot.isSimulation()){
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(Constants.tagLayout);
      var camProp = new SimCameraProperties();
      camProp.setCalibration(1920, 1080, Rotation2d.fromDegrees(120));
      camProp.setFPS(20);
      camProp.setCalibError(0.25, 0.08);
      camProp.setAvgLatencyMs(35);
      camProp.setLatencyStdDevMs(5);
      leftCamSim = new PhotonCameraSim(leftCam);
      rightCamSim = new PhotonCameraSim(rightCam);
      visionSim.addCamera(leftCamSim, Constants.leftCamPose);
      visionSim.addCamera(rightCamSim, Constants.rightCamPose);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftResults = estimatePose(leftCam, leftCamSim, leftPoseEstimator);
    rightResults = estimatePose(rightCam, rightCamSim, rightPoseEstimator);
    /*
    RobotContainer.logger.updateCameraDetections();
    if(Robot.isSimulation()){
      visionSim.update(RobotContainer.drivetrain.getPose());
    }
    */
    
  }

  public List<PhotonPipelineResult> estimatePose(PhotonCamera cam, PhotonCameraSim camSim, PhotonPoseEstimator estimator){
    List<PhotonPipelineResult> results = Robot.isReal() ? cam.getAllUnreadResults() : camSim.getCamera().getAllUnreadResults();
    for(PhotonPipelineResult result : results){
      Optional<EstimatedRobotPose> maybeEst = estimator.estimateCoprocMultiTagPose(result);
      if(maybeEst.isEmpty()) //there may be only one tag seen; switch to single-tag estimation
        maybeEst = estimator.estimateLowestAmbiguityPose(result);
      
      if(maybeEst.isEmpty()) //no tags found still, so move on to next result
        continue;
      EstimatedRobotPose est = maybeEst.get();
      //TODO add camera std dev
      // RobotContainer.drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
    }
    return results;
  }
}
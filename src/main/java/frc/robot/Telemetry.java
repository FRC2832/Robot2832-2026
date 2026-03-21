package frc.robot;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct)
            .publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
            .getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
            .publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* AprilTag detections from PhotonVision */
    private final NetworkTable visionTable = inst.getTable("VisionData");
    private final NetworkTable leftCamTable = visionTable.getSubTable("LeftCamera");
    private final NetworkTable rightCamTable = visionTable.getSubTable("RightCamera");
    private final StructArrayPublisher<Pose3d> leftCamDetects = leftCamTable
            .getStructArrayTopic("TagPoses", Pose3d.struct).publish();
    private final IntegerArrayPublisher leftCamIds = leftCamTable.getIntegerArrayTopic("FiducialIDs").publish();
    private final StructArrayPublisher<Pose3d> rightCamDetects = rightCamTable
            .getStructArrayTopic("TagPoses", Pose3d.struct).publish();
    private final IntegerArrayPublisher rightCamIds = rightCamTable.getIntegerArrayTopic("FiducialIDs").publish();
    public final StructPublisher<Pose2d> estimatedRobotPose = visionTable
            .getStructTopic("EstimatedPose", Pose2d.struct).publish();

    private final NetworkTable shooterTable = inst.getTable("ShooterData");
    private final NetworkTable leftShooter = shooterTable.getSubTable("Left");
    private final NetworkTable rightShooter = shooterTable.getSubTable("Right");
    private final NetworkTable accelerator = shooterTable.getSubTable("Accelerator");
    public final DoublePublisher leftShooterSpeed = leftShooter.getDoubleTopic("Speed").publish();
    public final DoublePublisher rightShooterSpeed = rightShooter.getDoubleTopic("Speed").publish();
    public final DoublePublisher acceleratorSpeed = accelerator.getDoubleTopic("Speed").publish();
    public final BooleanPublisher leftShooterAtSpeed = leftShooter.getBooleanTopic("AtSpeed").publish();
    public final BooleanPublisher rightShooterAtSpeed = rightShooter.getBooleanTopic("AtSpeed").publish();
    public final BooleanPublisher acceleratorAtSpeed = accelerator.getBooleanTopic("AtSpeed").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                            new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                            new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                            new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                            new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    /**
     * Accept the swerve drive state and telemeterize it to SmartDashboard and
     * SignalLogger.
     */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct,
                state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct,
                state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }

    public void updateAprilTagDetections(List<PhotonPipelineResult> left, List<PhotonPipelineResult> right) {
        for (var result : left) {
            var targets = result.getTargets();
            Pose3d[] targetsArr = new Pose3d[targets.size()];
            long[] targetIds = new long[targetsArr.length];
            for (int i = 0; i < targetsArr.length; i++) {
                targetIds[i] = targets.get(i).getFiducialId();
                targetsArr[i] = Constants.TAG_LAYOUT.getTagPose((int) targetIds[i])
                        .orElse(Pose3d.kZero);
            }
            long timestampMicros = (long) (1000000 * result.getTimestampSeconds());
            leftCamDetects.set(targetsArr, timestampMicros);
            leftCamIds.set(targetIds, timestampMicros);
        }
        for (var result : right) {
            var targets = result.getTargets();
            Pose3d[] targetsArr = new Pose3d[targets.size()];
            long[] targetIds = new long[targetsArr.length];
            for (int i = 0; i < targetsArr.length; i++) {
                targetIds[i] = targets.get(i).getFiducialId();
                targetsArr[i] = Constants.TAG_LAYOUT.getTagPose((int) targetIds[i])
                        .orElse(Pose3d.kZero);
            }
            long timestampMicros = (long) (1000000 * result.getTimestampSeconds());
            rightCamDetects.set(targetsArr, timestampMicros);
            rightCamIds.set(targetIds, timestampMicros);
        }
    }
}

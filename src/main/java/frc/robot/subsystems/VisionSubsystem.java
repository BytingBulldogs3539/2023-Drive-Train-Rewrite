package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionSubsystem extends Thread {
    private Notifier m_telemetry;

    // Vision Variables
    AprilTagFieldLayout aprilTagFieldLayout;

    public PhotonCamera leftCam;
    Transform3d robotToLeftCam = new Transform3d(
            new Translation3d(-0.1746 - .07 + .02 + 0.08, 0.2885 + 0.05 - .03, 0.3876),
            new Rotation3d(Math.toRadians(0), 0, Math.toRadians(14)));

    public PhotonCamera rightCam;
    Transform3d robotToRightCam = new Transform3d(
            new Translation3d(-0.1746 + .05 + .02, -0.2885 - .1 + .05, 0.3876),
            new Rotation3d(Math.toRadians(0), 0, Math.toRadians(-14)));

    PhotonPoseEstimator leftPhotonPoseEstimator;
    PhotonPoseEstimator rightPhotonPoseEstimator;
    Optional<EstimatedRobotPose> resultLeft;
    Optional<EstimatedRobotPose> resultRight;
    boolean useVision = true;
    double leftLastTimeStamp = 0;
    double rightLastTimeStamp = 0;

    DriveSubsystem driveSub;

    public VisionSubsystem(DriveSubsystem driveSub) {
        super();
        this.driveSub = driveSub;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

        } catch (Exception e) {
            System.out.println("ERROR Loading April Tag DATA");
            aprilTagFieldLayout = null;
        }

        leftCam = new PhotonCamera("LeftCam");
        leftPhotonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP,
                leftCam,
                robotToLeftCam);
        rightCam = new PhotonCamera("RightCam");
        rightPhotonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP,
                rightCam,
                robotToRightCam);
        setVisionWeights(.2, .2, 10);
    }

    // Vision Methods
    public Optional<EstimatedRobotPose> getEstimatedLeftGlobalPose() {
        return leftPhotonPoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedRightGlobalPose() {
        return rightPhotonPoseEstimator.update();
    }

    public void useVision(boolean useVision) {
        this.useVision = useVision;
    }

    public void setVisionWeights(double visionX, double visionY, double visionDeg) {
        driveSub.swerveController.m_odometry.setVisionMeasurementStdDevs(
                VecBuilder.fill(visionX, visionY, Units.degreesToRadians(visionDeg)));
    }

    public void setStartPosition(DriveSubsystem.StartPosition position) {
        // Configure which camera to use in auton based on start position
        switch (position) {
            case RED_SMOOTH:
                aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                rightCam.setDriverMode(false);
                leftCam.setDriverMode(true);
                break;
            case RED_CABLE:
                aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                rightCam.setDriverMode(true);
                leftCam.setDriverMode(false);
                break;
            case BLUE_SMOOTH:
                aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                rightCam.setDriverMode(true);
                leftCam.setDriverMode(false);
                break;
            case BLUE_CABLE:
                aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                rightCam.setDriverMode(false);
                leftCam.setDriverMode(true);
                break;
        }
        // Update the april tag layout based on our start position
        leftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
        rightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
    }

    Alliance lastAlliance = Alliance.Invalid;

    @Override
    public void run() {
        /* Run as fast as possible, our signals will control the timing */
        while (true) {
            Logger logger = Logger.getInstance();

            // Vision Calculations

            if (DriverStation.getAlliance() == Alliance.Blue && lastAlliance!=Alliance.Blue)
                aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                leftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
                rightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

            if (DriverStation.getAlliance() == Alliance.Red && lastAlliance!=Alliance.Red)
                aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                leftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
                rightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

            this.resultLeft = getEstimatedLeftGlobalPose();
            this.resultRight = getEstimatedRightGlobalPose();

            if (useVision) {

                double visioncutoff = 4;
                if (resultLeft.isPresent()) {
                    EstimatedRobotPose camPoseLeft = resultLeft.get();
                    if (camPoseLeft.timestampSeconds != leftLastTimeStamp) {
                        logger.recordOutput("/DriveTrain/LeftCamPose", camPoseLeft.estimatedPose.toPose2d());
                        synchronized (driveSub.swerveController.m_odometry) {
                            
                            if (camPoseLeft.estimatedPose.toPose2d().getX() < visioncutoff) {
                                driveSub.swerveController.m_odometry.addVisionMeasurement(
                                        camPoseLeft.estimatedPose.toPose2d(), camPoseLeft.timestampSeconds);


                            }
                        }
                    }
                    leftLastTimeStamp = camPoseLeft.timestampSeconds;
                }
                if (resultRight.isPresent()) {
                    EstimatedRobotPose camPoseRight = resultRight.get();
                    if (camPoseRight.timestampSeconds != rightLastTimeStamp) {
                        logger.recordOutput("/DriveTrain/RightCamPose", camPoseRight.estimatedPose.toPose2d());
                        synchronized (driveSub.swerveController.m_odometry) {
                            if (camPoseRight.estimatedPose.toPose2d().getX() < visioncutoff) {
                                driveSub.swerveController.m_odometry.addVisionMeasurement(
                                        camPoseRight.estimatedPose.toPose2d(), camPoseRight.timestampSeconds);
                            }
                        }
                    }
                    rightLastTimeStamp = camPoseRight.timestampSeconds;
                }
            }
        }
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.frcteam3539.CTRE_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.CTRE_Swerve_Lib.control.PidConstants;
import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.CTRSwerveDrivetrain;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.CTRSwerveModule;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.SwerveDriveConstantsCreator;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.SwerveDriveTrainConstants;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.SwerveModuleConstants;
import org.frcteam3539.CTRE_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.CTRE_Swerve_Lib.util.HolonomicFeedforward;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenixpro.configs.Slot0Configs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IDConstants;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new DriveSubsystem. */

	public enum StartPosition {
		RED_SMOOTH,
		RED_CABLE,
		BLUE_SMOOTH,
		BLUE_CABLE
	}

	ShuffleboardTab tab;
	public CTRSwerveDrivetrain swerveController;
	private final HolonomicMotionProfiledTrajectoryFollower follower;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


	public DriveSubsystem() {
	
		// LoggedPowerDistribution.getInstance(moduleId, moduleType)
		tab = Shuffleboard.getTab("Drivetrain");

		Slot0Configs steerGains = new Slot0Configs();
		Slot0Configs driveGains = new Slot0Configs();
		{
			steerGains.kP = DriveConstants.steerkP;
			steerGains.kI = DriveConstants.steerkI;
			steerGains.kD = DriveConstants.steerkD;

			driveGains.kP = DriveConstants.drivekP;
			driveGains.kI = DriveConstants.drivekI;
			driveGains.kD = DriveConstants.drivekD;
		}

		SwerveDriveConstantsCreator m_constantsCreator = new SwerveDriveConstantsCreator(
				DriveConstants.driveGearReduction,
				DriveConstants.steerGearReduction,
				DriveConstants.wheelDiameter / 2.0,
				DriveConstants.swerveModuleSlipCurrent,
				steerGains, // Use the specified steer gains
				driveGains, // Use the specified drive gains
				true // CANcoder not reversed from the steer motor. For WCP Swerve X this should be
						// true.
		);

		SwerveDriveTrainConstants driveTrainConstants = new SwerveDriveTrainConstants()
				.withCANbusName(IDConstants.swerveCanbusName)
				.withPigeon2Id(IDConstants.PigeonID)
				.withTurnKp(DriveConstants.RotationkP)
				.withTurnKd(DriveConstants.RotationkD);

		/**
		 * Note: WPI's coordinate system is X forward, Y to the left so make sure all
		 * locations are with
		 * respect to this coordinate system
		 *
		 */
		SwerveModuleConstants frontLeft = m_constantsCreator.createModuleConstants(
				IDConstants.FLSteeringID, IDConstants.FLDriveID, IDConstants.FLCanCoderID, DriveConstants.FLSteerOffset,
				DriveConstants.wheelTrackLength / 2.0, DriveConstants.wheelTrackWidth / 2.0);

		SwerveModuleConstants frontRight = m_constantsCreator.createModuleConstants(
				IDConstants.FRSteeringID, IDConstants.FRDriveID, IDConstants.FRCanCoderID, DriveConstants.FRSteerOffset,
				DriveConstants.wheelTrackLength / 2.0, -DriveConstants.wheelTrackWidth / 2.0);

		SwerveModuleConstants backRight = m_constantsCreator.createModuleConstants(
				IDConstants.BRSteeringID, IDConstants.BRDriveID, IDConstants.BRCanCoderID, DriveConstants.BRSteerOffset,
				-DriveConstants.wheelTrackLength / 2.0, -DriveConstants.wheelTrackWidth / 2.0);

		SwerveModuleConstants backLeft = m_constantsCreator.createModuleConstants(
				IDConstants.BLSteeringID, IDConstants.BLDriveID, IDConstants.BLCanCoderID, DriveConstants.BLSteerOffset,
				-DriveConstants.wheelTrackLength / 2.0, DriveConstants.wheelTrackWidth / 2.0);

		swerveController = new CTRSwerveDrivetrain(tab, driveTrainConstants, frontLeft, frontRight, backLeft,
				backRight);


		DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
				DriveConstants.TranslationkV,
				DriveConstants.TranslationkA, DriveConstants.TranslationkS);

		follower = new HolonomicMotionProfiledTrajectoryFollower(
				new PidConstants(DriveConstants.TranslationkP, DriveConstants.TranslationkI,
						DriveConstants.TranslationkD),
				new PidConstants(DriveConstants.RotationkP, DriveConstants.RotationkI, DriveConstants.RotationkD),
				new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

		/*tab.addNumber("Requested X", () -> {
			if (follower.getLastState() != null) {
				return follower.getLastState().getPathState().getPose2d().getX();
			}
			return 0.0;
		});
		tab.addNumber("Requested Y", () -> {
			if (follower.getLastState() != null) {
				return follower.getLastState().getPathState().getPose2d().getY();
			}
			return 0.0;
		});

		tab.addNumber("Chassis X Speed", () -> m_chassisSpeeds.vxMetersPerSecond);

		tab.addNumber("Chassis Y Speed", () -> m_chassisSpeeds.vyMetersPerSecond);*/

		setDefaultCommand(new DriveCommand(this));

		resetRobotPose(new Pose2d(new Translation2d(),Rotation2d.fromDegrees(180)));
	}

	public HolonomicMotionProfiledTrajectoryFollower getFollower() {
		return follower;
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	public void resetRobotPose(Trajectory trajectory) {
		swerveController.resetPosition(trajectory.calculate(0.0).getPathState().getPose2d());
	}

	public void resetRobotPose(Pose2d pose) {
		swerveController.resetPosition(pose);
	}

	public void resetRobotRotation(Rotation2d rot) {
		swerveController.resetPosition(new Pose2d(swerveController.getPoseMeters().getTranslation(), rot));
		
	}

	public Pose2d getPose2d() {
		return swerveController.getPoseMeters();
	}

	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(swerveController.getPigeon2().getYaw().getValue());
	}

	public double getPitch() {
		return swerveController.getPigeon2().getRoll().getValue();
	}

	public void zeroModulesOffsets()
	{
		for (CTRSwerveModule module: swerveController.getModules()) {
			module.setCANcoderOffset(0.0);
		}
	}
	/**
	 * Must call zero modules offset first.
	 */
	public void saveModuleOffsets()
	{
		DriveConstants.FLSteerOffset = -swerveController.getModules()[0].getPosition(false).angle.getRotations();
		DriveConstants.FRSteerOffset = -swerveController.getModules()[1].getPosition(false).angle.getRotations();
		DriveConstants.BLSteerOffset = -swerveController.getModules()[2].getPosition(false).angle.getRotations();
		DriveConstants.BRSteerOffset = -swerveController.getModules()[3].getPosition(false).angle.getRotations();
		RobotContainer.driveConstants.save();
		swerveController.getModules()[0].setCANcoderOffset(DriveConstants.FLSteerOffset);
		swerveController.getModules()[1].setCANcoderOffset(DriveConstants.FRSteerOffset);
		swerveController.getModules()[2].setCANcoderOffset(DriveConstants.BLSteerOffset);
		swerveController.getModules()[3].setCANcoderOffset(DriveConstants.BRSteerOffset);
	}

	@Override
	public void periodic() {
		var driveSignalOpt = follower.update(swerveController.getPoseMeters(), Timer.getFPGATimestamp(),
				Robot.defaultPeriodSecs);
		// If we should be running a profile use those chassisspeeds instead.
		if (driveSignalOpt.isPresent()) {
			m_chassisSpeeds = driveSignalOpt.get();
		}

		swerveController.driveRobotCentric(m_chassisSpeeds);
	}

	public void log() {
		Logger logger = Logger.getInstance();
		
		Pose2d trajectory = follower.getLastState() != null?
			follower.getLastState().getPathState().getPose2d() : new Pose2d(-1, -1, new Rotation2d(0));
		logger.recordOutput("/DriveTrain/Trajectory", trajectory);

		logger.recordOutput("/DriveTrain/Odometry", swerveController.getPoseMeters());
		
		logger.recordOutput(
			"/DriveTrain/RequestedChassisSpeeds",
			new double[] { 
				m_chassisSpeeds.vxMetersPerSecond,
				m_chassisSpeeds.vyMetersPerSecond,
				m_chassisSpeeds.omegaRadiansPerSecond
			}
		);
	}
}

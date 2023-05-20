// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.frcteam3539.CTRE_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.CTRE_Swerve_Lib.control.Path;
import org.frcteam3539.CTRE_Swerve_Lib.control.PidConstants;
import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.CTRSwerveDrivetrain;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.SwerveDriveConstantsCreator;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.SwerveDriveTrainConstants;
import org.frcteam3539.CTRE_Swerve_Lib.swerve.SwerveModuleConstants;
import org.frcteam3539.CTRE_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.CTRE_Swerve_Lib.util.HolonomicFeedforward;

import com.ctre.phoenixpro.configs.Slot0Configs;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IDConstants;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new DriveSubsystem. */
	ShuffleboardTab tab;
	public CTRSwerveDrivetrain swerveController;
	private final HolonomicMotionProfiledTrajectoryFollower follower;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


	public DriveSubsystem() {
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
				DriveConstants.TranslationkV, DriveConstants.TranslationkS);

		follower = new HolonomicMotionProfiledTrajectoryFollower(
				new PidConstants(DriveConstants.TranslationkP, DriveConstants.TranslationkI, DriveConstants.TranslationkD), new PidConstants(DriveConstants.RotationkP, DriveConstants.RotationkI, DriveConstants.RotationkD),
				new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

		setDefaultCommand(new DriveCommand(this));
	}

	public HolonomicMotionProfiledTrajectoryFollower getFollower()
	{
		return follower;
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}
	public void resetRobotPose(Trajectory trajectory)
	{
		swerveController.resetPosition(trajectory.calculate(0.0).getPathState().getPose2d());
	}

	@Override
	public void periodic() {
		ChassisSpeeds chassisSpeeds = m_chassisSpeeds;
		var driveSignalOpt = follower.update(swerveController.getPoseMeters(), Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

		//If we should be running a profile use those chassisspeeds instead.
        if (driveSignalOpt.isPresent()) {
            chassisSpeeds = driveSignalOpt.get();
			//swerveController.driveRobotCentric(chassisSpeeds);
        }

		swerveController.driveRobotCentric(chassisSpeeds);
	}
}

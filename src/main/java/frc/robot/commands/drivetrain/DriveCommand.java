// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
	/** Creates a new DriveCommand. */
	DriveSubsystem driveSubsystem;

	public DriveCommand(DriveSubsystem driveSubsystem) {
		addRequirements(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Transform sticks to translation2d while matching joystick xy to field xy
		// (they are opposite)
		Translation2d joystickRight = new Translation2d(-RobotContainer.driverController.getRightY(),
				-RobotContainer.driverController.getRightX());
		Translation2d joystickLeft = new Translation2d(-RobotContainer.driverController.getLeftY(),
				-RobotContainer.driverController.getLeftX());

		joystickRight = modifyJoystick(joystickRight);
		joystickLeft = modifyJoystick(joystickLeft);
		Rotation2d rot;
		if (RobotContainer.driverController.rightBumper().getAsBoolean()) {
			rot = Rotation2d.fromDegrees(0);
		} else {
			rot = RobotContainer.driveSubsystem.getPose2d().getRotation();
		}
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				joystickLeft.getX() * RobotContainer.driveSubsystem.swerveController.getMaxVelocity()
						* DriveConstants.driveSpeedMultiplier,
				joystickLeft.getY() * RobotContainer.driveSubsystem.swerveController.getMaxVelocity()
						* DriveConstants.driveSpeedMultiplier,
				joystickRight.getY() * RobotContainer.driveSubsystem.swerveController.getMaxRotationVelocity()
						* DriveConstants.rotationSpeedMultiplier,
				rot);

		if (RobotContainer.driverController.rightTrigger(0.5).getAsBoolean()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
					joystickLeft.getX() * RobotContainer.driveSubsystem.swerveController.getMaxVelocity(),
					joystickLeft.getY() * RobotContainer.driveSubsystem.swerveController.getMaxVelocity(),
					joystickRight.getY() * RobotContainer.driveSubsystem.swerveController.getMaxRotationVelocity()
							* DriveConstants.turboRotationSpeedMultiplier,
					rot);
		}
		if (RobotContainer.driverController.leftTrigger(0.5).getAsBoolean()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
					joystickLeft.getX() * RobotContainer.driveSubsystem.swerveController.getMaxVelocity()
							* DriveConstants.slowSpeedMultiplier,
					joystickLeft.getY() * RobotContainer.driveSubsystem.swerveController.getMaxVelocity()
							* DriveConstants.slowSpeedMultiplier,
					joystickRight.getY() * RobotContainer.driveSubsystem.swerveController.getMaxRotationVelocity()
							* DriveConstants.slowRotationSpeedMultiplier,
					rot);

		}

		driveSubsystem.drive(chassisSpeeds);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static Translation2d modifyJoystick(Translation2d joystick) {
		// Deadband
		joystick = new Translation2d(deadband(joystick.getX(), 0.05), deadband(joystick.getY(), 0.05));

		Rotation2d rotation = joystick.getAngle();
		double distance = joystick.getNorm();

		double distanceModified = Math.copySign(Math.pow(distance, 3), distance);

		return new Translation2d(distanceModified, rotation);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

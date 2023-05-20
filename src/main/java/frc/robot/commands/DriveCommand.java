// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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
    //Transform sticks to translation2d while matching joystick xy to field xy (they are opposite)
    Translation2d joystickRight = new Translation2d(-RobotContainer.driverController.getRightX(), -RobotContainer.driverController.getRightY());
    Translation2d joystickLeft = new Translation2d(-RobotContainer.driverController.getLeftX(), -RobotContainer.driverController.getLeftY());

    joystickRight = modifyJoystick(joystickRight);
    joystickLeft = modifyJoystick(joystickLeft);

    System.out.println(joystickRight.getX() + " " + joystickRight.getY());
    System.out.println(joystickLeft.getX() + " " + joystickLeft.getY());

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(joystickLeft.getX(), joystickLeft.getY(), joystickRight.getX(),Rotation2d.fromDegrees(driveSubsystem.swerveController.getPigeon2().getYaw().getValue()));

    System.out.println("Chassis "+chassisSpeeds.vxMetersPerSecond+" " + chassisSpeeds.vyMetersPerSecond);
    
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
    joystick = new Translation2d(deadband(joystick.getX(), 0.05),deadband(joystick.getY(), 0.05));

    Rotation2d rotation = joystick.getAngle();
    double distance = joystick.getDistance(new Translation2d());
    
    double distanceModified = Math.copySign(Math.pow(distance, 3), distance);

    return new Translation2d(distanceModified, rotation);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

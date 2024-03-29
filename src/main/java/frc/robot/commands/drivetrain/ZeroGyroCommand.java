// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ZeroGyroCommand extends CommandBase {
  DriveSubsystem driveSub;
  public ZeroGyroCommand(DriveSubsystem driveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.resetRobotRotation(Rotation2d.fromDegrees(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

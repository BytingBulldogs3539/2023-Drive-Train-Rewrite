// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import org.frcteam3539.CTRE_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.CTRE_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.CTRE_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory;
import org.frcteam3539.CTRE_Swerve_Lib.control.TrajectoryConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Follow_Simple_Line extends SequentialCommandGroup {
  /** Creates a new Follow_Simple_Line. */
  Trajectory traj1 = new Trajectory(
      new SimplePathBuilder(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)))
          .lineTo(new Translation2d(1, 0)).build(),
      getConstraints(),
      0.05);

  public Follow_Simple_Line() {
    addCommands(
        new InstantCommand(() -> RobotContainer.driveSubsystem.resetRobotPose(traj1)),
        new FollowTrajectory(RobotContainer.driveSubsystem, traj1));
  }

  public TrajectoryConstraint[] getConstraints() {
    TrajectoryConstraint[] constraints = { (TrajectoryConstraint) new MaxAccelerationConstraint(1),
        (TrajectoryConstraint) new MaxVelocityConstraint(2) };
    return constraints;
  }
}

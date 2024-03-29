// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.MPLoader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SplineAuton extends SequentialCommandGroup {
	/** Creates a new SplineAuton. */
	MPLoader loader = new MPLoader("test.txt", false);
	private Command[] sequence = {
			// Setup
			new InstantCommand(() -> RobotContainer.driveSubsystem.resetRobotPose(loader.getFirstTrajectory())),
			new FollowTrajectory(RobotContainer.driveSubsystem, loader.getNextTrajectory()),
			new FollowTrajectory(RobotContainer.driveSubsystem, loader.getNextTrajectory()),
			new FollowTrajectory(RobotContainer.driveSubsystem, loader.getNextTrajectory())

			//new FollowTrajectory(RobotContainer.driveSubsystem, loader.getNextTrajectory()),
			//new FollowTrajectory(RobotContainer.driveSubsystem, loader.getNextTrajectory())
	};

	public SplineAuton() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(sequence);
	}
}

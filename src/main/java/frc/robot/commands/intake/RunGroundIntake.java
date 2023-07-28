// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class RunGroundIntake extends CommandBase {
	/** Creates a new RunGroundIntake. */
	IntakeSubsystem intakeSub;
	public RunGroundIntake(IntakeSubsystem intakeSub) {
		this.intakeSub = intakeSub;
		addRequirements(intakeSub);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		intakeSub.setGroundIntakeSpeed(RobotContainer.operatorController.getLeftY(),
				RobotContainer.operatorController.getRightY());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intakeSub.setGroundIntakeSpeed(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

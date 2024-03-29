// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class FlipArmSideCommand extends CommandBase {
  /** Creates a new FlipArmSideCommand. */
  ArmSubsystem elevSub;

  public FlipArmSideCommand(ArmSubsystem elevSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevSub = elevSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevSub.getSide() == ArmSubsystem.Sides.front)
      elevSub.setSide(ArmSubsystem.Sides.back);
    else if (elevSub.getSide() == ArmSubsystem.Sides.back)
      elevSub.setSide(ArmSubsystem.Sides.front);
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

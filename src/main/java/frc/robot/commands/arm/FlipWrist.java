// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class FlipWrist extends CommandBase {
  /** Creates a new FlipWrist. */
  ArmSubsystem elevSub;
  LEDSubsystem ledSub;

  public FlipWrist(ArmSubsystem elevSub, LEDSubsystem ledSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevSub = elevSub;
    this.ledSub = ledSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevSub.getWristOrientation() == ArmSubsystem.Wrist.cube) {
      elevSub.setWristOrientation(ArmSubsystem.Wrist.cone);
      this.ledSub.setLEDs(LEDState.CONE);
    } else if (elevSub.getWristOrientation() == ArmSubsystem.Wrist.cone) {
      elevSub.setWristOrientation(ArmSubsystem.Wrist.cube);
      this.ledSub.setLEDs(LEDState.CUBE);
    }
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

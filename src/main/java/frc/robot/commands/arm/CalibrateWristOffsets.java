// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateWristOffsets extends CommandBase {
  /** Creates a new CalibrateWheelOffsets. */
  ArmSubsystem armSub;
  public CalibrateWristOffsets(ArmSubsystem armSub) {
    this.armSub = armSub;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  boolean isFinished = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (DriverStation.isEnabled()) {
      isFinished = true;
      DriverStation.reportError("Robot must be disabled to calibrate the wrist", false);
    }
    if (DriverStation.isDisabled()) {
      armSub.zeroWristOffset();
      armSub.setWristBreakMode(false);
      
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (DriverStation.isDisabled()) {
      armSub.saveWristRotationOffset();
      DriverStation.reportWarning("Wrist Offsets Saved", false);

    }
    armSub.setWristBreakMode(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

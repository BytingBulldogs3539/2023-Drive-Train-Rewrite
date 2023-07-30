// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CalibrateWheelOffsets extends CommandBase {
  /** Creates a new CalibrateWheelOffsets. */
  public CalibrateWheelOffsets() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  boolean isFinshed = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (DriverStation.isEnabled()) {
      isFinshed = true;
      DriverStation.reportError("Robot must be disabled to calibrate wheels", false);
    }
    if (DriverStation.isDisabled()) {
      RobotContainer.driveSubsystem.zeroModulesOffsets();
      this.withTimeout(.5);
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
      RobotContainer.driveSubsystem.saveModuleOffsets();
      DriverStation.reportWarning("Wheel Offsets Saved", false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished();
  }
}

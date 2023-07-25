// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.autons.*;
import frc.robot.constants.*;

public class RobotContainer {
	public static DriveConstants driveConstants = new DriveConstants();
	public static IDConstants idConstants = new IDConstants();

	public static DriveSubsystem driveSubsystem = new DriveSubsystem();

	public static CommandXboxController driverController = new CommandXboxController(1);

	public static SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		putAutons();
		configureBindings();
	}

	private void putAutons() {
		chooser.setDefaultOption("Follow Simple Line", new Follow_Simple_Line());
		SmartDashboard.putData(chooser);
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}

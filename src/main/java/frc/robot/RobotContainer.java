// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.Arm;
import frc.robot.subsystems.ArmSubsystem.Sides;
import frc.robot.subsystems.ArmSubsystem.Wrist;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.utilities.LogController;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.misc.*;
import frc.robot.autons.*;
import frc.robot.constants.*;

public class RobotContainer {
	public static DriveConstants driveConstants = new DriveConstants();
	public static IDConstants idConstants = new IDConstants();
	public static ElevatorConstants elevatorConstants = new ElevatorConstants();

	public static DriveSubsystem driveSubsystem = new DriveSubsystem();
	public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public static ArmSubsystem armSubsystem = new ArmSubsystem();
	public static LEDSubsystem ledSubsystem = new LEDSubsystem(true, armSubsystem);
	public static VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);

	public static CommandXboxController driverController = new CommandXboxController(1);
	public static CommandXboxController operatorController = new CommandXboxController(0);

	public static SendableChooser<Command> chooser = new SendableChooser<Command>();

	public static LogController logController = new LogController(
			driveSubsystem, armSubsystem, intakeSubsystem, ledSubsystem);

	public RobotContainer() {

		DriverStation.silenceJoystickConnectionWarning(true);
		putAutons();
		configureBindings();
		visionSubsystem.start();
	}

	private void putAutons() {
		chooser.addOption("Follow Simple Line", new Follow_Simple_Line());
		chooser.setDefaultOption("Follow Simple Spline", new SplineAuton());
		chooser.addOption("BlueConeCubeHigh", new BlueConeCubeHigh());
		chooser.addOption("RedConeCubeHigh", new RedConeCubeHigh());
		chooser.addOption("BlueHighConeBack", new BlueHighConeBack());
		chooser.addOption("RedHighConeBack", new RedHighConeBack());
		chooser.addOption("LeftRedHighConeBack", new LeftRedHighConeBack());
		chooser.addOption("RightBlueHighConeBack", new RightBlueHighConeBack());
		chooser.addOption("BlueConeBackBalance", new BlueConeBackBalance());
		chooser.addOption("RedConeBackBalance", new RedConeBackBalance());
		chooser.addOption("Blue2_5", new Blue2_5());
		chooser.addOption("Red2_5", new Red2_5());
		// chooser.addOption("Blue2_Balance", new Blue2_Balance());
		chooser.addOption("BlueConeOverBalance", new BlueConeOverBalance());
		chooser.addOption("RedConeOverBalance", new RedConeOverBalance());

		SmartDashboard.putData(chooser);
	}

	private void configureBindings() {
		SmartDashboard.putData(new CalibrateWheelOffsets().ignoringDisable(true));
		SmartDashboard.putData(new CalibrateArmOffsets(armSubsystem).ignoringDisable(true));
		SmartDashboard.putData(new CalibrateWristOffsets(armSubsystem).ignoringDisable(true));

		driverController.start().onTrue(new ZeroGyroCommand(driveSubsystem));

		operatorController.leftTrigger()
				.whileTrue(new AutomaticIntakeCommand(intakeSubsystem, armSubsystem, ledSubsystem, true));
		operatorController.rightTrigger()
				.whileTrue(new AutomaticIntakeCommand(intakeSubsystem, armSubsystem, ledSubsystem, false));

		// operatorController.leftTrigger()
		// .whileTrue(new IntakeCommand(intakeSubsystem, armSubsystem, ledSubsystem, 1,
		// false));

		// operatorController.rightTrigger()
		// .whileTrue(new IntakeCommand(intakeSubsystem, armSubsystem, ledSubsystem, -1,
		// false));

		operatorController.leftBumper().onTrue(new SetArmSide(armSubsystem, Sides.front));

		driverController.y().onTrue(new SetLEDs(ledSubsystem, LEDState.CONE));
		driverController.x().onTrue(new SetLEDs(ledSubsystem, LEDState.CUBE));
		// driverController.b().whileTrue(new AutoPoleAlign(ledSubsystem));

		operatorController.a().onTrue(new SetArmHeight(armSubsystem, Arm.intake));
		operatorController.b().onTrue(new SetArmHeight(armSubsystem, Arm.low));
		operatorController.y().onTrue(new SetArmHeight(armSubsystem, Arm.middle));
		operatorController.x().onTrue(new SetArmHeight(armSubsystem, Arm.high));
		operatorController.povRight().onTrue(new SetArmHeight(armSubsystem, Arm.HumanPlayer));
		operatorController.povDown().onTrue(new ConfigureArm(armSubsystem, Sides.front, Arm.groundIntake, Wrist.cone));
		operatorController.povUp().onTrue(new SetArmHeight(armSubsystem, Arm.cubeLowIntake));

		operatorController.rightBumper().onTrue(new FlipWrist(armSubsystem, ledSubsystem));

		SmartDashboard.putData(new DisableBreakMode(armSubsystem));
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}

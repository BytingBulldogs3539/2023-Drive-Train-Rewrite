// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.LEDSubsystem.LEDState;
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
	public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public LEDSubsystem ledSubsystem = new LEDSubsystem(true, elevatorSubsystem);

	public static CommandXboxController driverController = new CommandXboxController(1);
	public static CommandXboxController operatorController = new CommandXboxController(0);


	public static SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		putAutons();
		configureBindings();
	}

	private void putAutons() {
		chooser.setDefaultOption("Follow Simple Line", new Follow_Simple_Line());
		chooser.setDefaultOption("Follow Simple Spline", new SplineAuton());

		SmartDashboard.putData(chooser);
	}

	private void configureBindings() {
		driverController.start().onTrue(new ZeroGyroCommand(driveSubsystem));

    operatorController.leftTrigger().whileTrue(new IntakeCommand(intakeSubsystem,elevatorSubsystem,ledSubsystem, 1,false));

    operatorController.rightTrigger().whileTrue(new IntakeCommand(intakeSubsystem,elevatorSubsystem,ledSubsystem,-1,false));

    operatorController.leftBumper().onTrue(new FlipArmSideCommand(elevatorSubsystem));

    driverController.y().onTrue(new SetLEDs(ledSubsystem, LEDState.CONE));
    driverController.x().onTrue(new SetLEDs(ledSubsystem, LEDState.CUBE));
    driverController.b().whileTrue(new AutoPoleAlign(ledSubsystem));

    operatorController.a().onTrue(new SetArmHeight(this.elevatorSubsystem, Arm.intake));
    operatorController.b().onTrue(new SetArmHeight(this.elevatorSubsystem,Arm.low));
    operatorController.y().onTrue(new SetArmHeight(this.elevatorSubsystem,Arm.middle));
    operatorController.x().onTrue(new SetArmHeight(this.elevatorSubsystem,Arm.high));
    operatorController.povRight().onTrue(new SetArmHeight(this.elevatorSubsystem,Arm.HumanPlayer));
    operatorController.povDown().onTrue(new SetArmHeight(this.elevatorSubsystem,Arm.groundIntake));
    operatorController.povUp().onTrue(new SetArmHeight(this.elevatorSubsystem,Arm.cubeLowIntake));

    //operatorController.a().onTrue(elevatorSubsystem.getArmTrajectoryFollower(new Point2D.Double(38.926915,-70.0)));
    //operatorController.b().onTrue(elevatorSubsystem.getArmTrajectoryFollower(new Point2D.Double(110.0,-1)));
    //operatorController.x().onTrue(elevatorSubsystem.getArmTrajectoryFollower(new Point2D.Double(-60.57,-78.12)));

    operatorController.rightBumper().onTrue(new FlipWrist(elevatorSubsystem));

    SmartDashboard.putData(new DisableBreakMode(elevatorSubsystem));
    //SmartDashboard.putData(new EnableLeftCamera());
    //SmartDashboard.putData(new EnableRightCamera());
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}

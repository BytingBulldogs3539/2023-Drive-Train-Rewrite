// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ConfigureArm;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.MPLoader;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.misc.SetVision;
import frc.robot.subsystems.ArmSubsystem.Arm;
import frc.robot.subsystems.ArmSubsystem.Sides;
import frc.robot.subsystems.ArmSubsystem.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueConeBackBalance extends SequentialCommandGroup {
    /** Creates a new SplineAuton. */
    MPLoader loader = new MPLoader("BlueConeBackBalace.txt", false);
    private Command[] sequence = {
            // Setup
            new SetVision(false),
            new InstantCommand(() -> RobotContainer.driveSubsystem.resetRobotPose(loader.getFirstTrajectory())),
            new ConfigureArm(RobotContainer.armSubsystem, Sides.front, Arm.high, Wrist.cone),
            new WaitCommand(2),
            new IntakeCommand(RobotContainer.intakeSubsystem, RobotContainer.armSubsystem, RobotContainer.ledSubsystem,
                    1)
                    .withTimeout(0.6),

            new ParallelCommandGroup(
                    new ConfigureArm(RobotContainer.armSubsystem, Sides.front, Arm.intake, Wrist.cube),
                    new WaitCommand(1.5),
                    new FollowTrajectory(RobotContainer.driveSubsystem, loader.getNextTrajectory())

            ),
            new AutoBalance(RobotContainer.driveSubsystem, 2.5).withTimeout(5)

    };

    public BlueConeBackBalance() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(sequence);
    }
}

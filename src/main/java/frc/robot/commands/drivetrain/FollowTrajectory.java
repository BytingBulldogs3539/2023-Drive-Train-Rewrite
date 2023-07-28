// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends CommandBase {

    private final DriveSubsystem drivetrain;

    private final Trajectory trajectory;

    public FollowTrajectory(DriveSubsystem drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().follow(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getFollower().getCurrentTrajectory().isEmpty();
    }
}

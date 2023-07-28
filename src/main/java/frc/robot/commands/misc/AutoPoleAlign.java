// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import org.frcteam3539.CTRE_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.CTRE_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.CTRE_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory;
import org.frcteam3539.CTRE_Swerve_Lib.control.TrajectoryConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class AutoPoleAlign extends CommandBase {
    /**
     * Wrapper command to generate a trajectory to the
     * nearest pole column
     */

    Command autoAlign;
    public final double X_DISTANCE = 1.7;
    public final double[] RED_POLES = {
            7.51, 6.40, 5.84, 4.72, 4.16, 3.05
    };
    public final double[] BLUE_POLES = {
            0.52, 1.63, 2.19, 3.31, 3.87, 4.98
    };

    LEDSubsystem ledSub;

    public AutoPoleAlign(LEDSubsystem ledSub) {
        this.ledSub = ledSub;
        this.autoAlign = null;
    }

    TrajectoryConstraint[] constraints = new TrajectoryConstraint[] {
            (TrajectoryConstraint) new MaxAccelerationConstraint(1),
            (TrajectoryConstraint) new MaxVelocityConstraint(1)
    };

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double robotY = RobotContainer.driveSubsystem.getPose2d().getY();
        double nearestY = -1;
        double smallestDist = 999999;

        // Find the nearest pole Y coordinate
        for (double y : DriverStation.getAlliance() == Alliance.Red ? RED_POLES : BLUE_POLES) {
            if (Math.abs(robotY - y) < smallestDist) {
                smallestDist = Math.abs(robotY - y);
                nearestY = y;
            }
        }

        // Generate trajectory command to nearest coordinate
        autoAlign = new FollowTrajectory(RobotContainer.driveSubsystem,
                new Trajectory(new SimplePathBuilder(RobotContainer.driveSubsystem.getPose2d())
                        .lineTo(
                                new Pose2d(new Translation2d(X_DISTANCE, nearestY), Rotation2d.fromDegrees(180)))
                        .build(), constraints, .02));

        // Indicate vision and start the trajectory command
        ledSub.saveState();
        ledSub.setLEDs(LEDState.CLIMBING);
        autoAlign.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!this.autoAlign.isFinished())
            this.autoAlign.cancel();
        ledSub.restoreState();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.autoAlign.isFinished();
    }
}

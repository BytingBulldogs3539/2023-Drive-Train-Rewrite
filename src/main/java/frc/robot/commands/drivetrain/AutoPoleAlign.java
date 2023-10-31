// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.frcteam3539.CTRE_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.CTRE_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.CTRE_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory;
import org.frcteam3539.CTRE_Swerve_Lib.control.TrajectoryConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class AutoPoleAlign extends CommandBase {
    /**
     * Wrapper command to generate a trajectory to the
     * nearest pole column
     */

    public final double X_DISTANCE = 1.80;
    public final double[] RED_POLES = {
            7.51, 6.40, 5.84, 4.72, 4.16, 3.05
    };
    public final double[] BLUE_POLES = {
            0.52, 1.63, 2.19, 3.31, 3.87, 4.98
    };

    public AutoPoleAlign() {
    }

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
        RobotContainer.driveSubsystem.getFollower().follow(new Trajectory(
                new SimplePathBuilder(RobotContainer.driveSubsystem.getPose2d())
                        .lineTo(
                                new Pose2d(X_DISTANCE, nearestY, Rotation2d.fromDegrees(180)))
                        .build(),
                new TrajectoryConstraint[] {
                        (TrajectoryConstraint) new MaxAccelerationConstraint(1),
                        (TrajectoryConstraint) new MaxVelocityConstraint(1)
                }, .05));

        // Indicate vision and start the trajectory command
        RobotContainer.ledSubsystem.saveState();
        RobotContainer.ledSubsystem.setLEDs(LEDState.CLIMBING);
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveSubsystem.getFollower().cancel();
        RobotContainer.ledSubsystem.restoreState();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.driveSubsystem.getFollower().getCurrentTrajectory().isEmpty();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.frcteam3539.CTRE_Swerve_Lib.control.*;
import org.frcteam3539.CTRE_Swerve_Lib.control.Path.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.*;

/** Add your docs here. */
public class ArmTrajectoryGenerator {
    double armLength;
    double maximumVelocity;
    double maximumAcceleration;
    double maximumArcVelcity;
    double maximumArcAcceleration;

    public ArmTrajectoryGenerator(double maximumVelocity, double maximumAcceleration, double maximumArcVelocity, double maximumArcAcceleration, double armLength) {
        this.maximumAcceleration = maximumAcceleration;
        this.maximumVelocity = maximumVelocity;
        this.armLength = armLength;
        this.maximumArcAcceleration = maximumArcAcceleration;
        this.maximumArcVelcity = maximumArcVelocity;
    }

    public MultiTrajectory generateTrajectories(Translation2d startPoint, Translation2d endPoint) {

        Translation2d retractPoint = new Translation2d(armLength, startPoint.getAngle());

        boolean clockwise = false;
        if (startPoint.getX() < 0 && endPoint.getX() > 0) {
            clockwise = true;
        }
        else if((startPoint.getX() < 0 && endPoint.getX() < 0) &&(startPoint.getY() > endPoint.getY()))
        {
            clockwise = false;
        }
        else if((startPoint.getX() < 0 && endPoint.getX() < 0) &&(startPoint.getY() < endPoint.getY()))
        {
            clockwise = true;
        }
        else if((startPoint.getX() > 0 && endPoint.getX() > 0) &&(startPoint.getY() < endPoint.getY()))
        {
            clockwise = false;
        }
        else if((startPoint.getX() > 0 && endPoint.getX() > 0) &&(startPoint.getY() > endPoint.getY()))
        {
            clockwise = true;
        }
        else if((startPoint.getX() == 0 && endPoint.getX() > 0))
        {
            clockwise = true;
        }
        else if((startPoint.getX() == 0 && endPoint.getX() < 0))
        {
            clockwise = true;
        }

        Translation2d extensionPoint = new Translation2d(armLength+.1, endPoint.getAngle());
        boolean sameQuad = sameQuadrant(startPoint, endPoint);


        TrajectoryConstraint[] constraints = {
            (TrajectoryConstraint) new MaxAccelerationConstraint(maximumAcceleration),
            (TrajectoryConstraint) new MaxVelocityConstraint(maximumVelocity),
            (TrajectoryConstraint) new CentripetalAccelerationConstraint(100) };

    TrajectoryConstraint[]  RotationConstraints = {
                (TrajectoryConstraint) new MaxAccelerationConstraint(maximumArcAcceleration),
                (TrajectoryConstraint) new MaxVelocityConstraint(maximumArcVelcity)};

        SimplePathBuilder p1 = new SimplePathBuilder(new Pose2d(startPoint, Rotation2d.fromDegrees(0)));
        Path path;
        if (sameQuad) {
            p1.lineTo(endPoint);
            path = p1.build();
            return new MultiTrajectory(new Trajectory(path, constraints, 3));
        } else {
            p1.lineTo(retractPoint);

            SimplePathBuilder p2 = new SimplePathBuilder(new Pose2d(retractPoint, Rotation2d.fromDegrees(0)));

            p2.arcTo(extensionPoint, new Translation2d(0,0), clockwise);

            SimplePathBuilder p3 = new SimplePathBuilder(new Pose2d(extensionPoint, Rotation2d.fromDegrees(0)));
            p3.lineTo(endPoint);

            Path path1 = p1.build();
            Path path2 = p2.build();
            //System.out.println(path2.getLength());
            //for (double i = 0; i < path2.getLength(); i += 0.02) {
            //    System.out.println(path2.calculate(i));
            //}
            Path path3 = p3.build();
            
            return new MultiTrajectory(new Trajectory(path1, constraints, 3),new Trajectory(path2, RotationConstraints, 3),new Trajectory(path3, constraints, 3));
            
        }
    }

    public static boolean sameQuadrant(Translation2d p1, Translation2d p2) {
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        if ((x1 > 0 && x2 > 0) && (y1 > 0 && y2 > 0)) {
            // both points are in quadrant 1
            return true;
        } else if ((x1 < 0 && x2 < 0) && (y1 > 0 && y2 > 0)) {
            // both points are in quadrant 2
            return true;
        } else if ((x1 < 0 && x2 < 0) && (y1 < 0 && y2 < 0)) {
            // both points are in quadrant 3
            return true;
        } else if ((x1 > 0 && x2 > 0) && (y1 < 0 && y2 < 0)) {
            // both points are in quadrant 4
            return true;
        } else {
            // the points are not in the same quadrant
            return false;
        }
    }

    public static void main(String[] args) {
        ArmTrajectoryGenerator h = new ArmTrajectoryGenerator(200, 200, 500, 500, 78.74);
        MultiTrajectory traj2 = h.generateTrajectories(
            
                new Translation2d(ElevatorConstants.frontCubeHighX, ElevatorConstants.frontCubeHighY),
                new Translation2d(ElevatorConstants.backCubeIntakeX, ElevatorConstants.backConeIntakeY)
                );

        ArrayList<Translation2d> points = new ArrayList<Translation2d>();

        for (double i = 0; i < traj2.getDuration(); i += 0.02) {

            State s = traj2.calculate(i).getPathState();

            Translation2d p = new Translation2d(s.getPose2d().getX(), s.getPose2d().getY());

            if (p.getAngle().getDegrees() < -95) {
                p = new Translation2d(p.getNorm(),p.getAngle().plus(Rotation2d.fromDegrees(360)));
            }
            points.add(p);
        }

        for (Translation2d point : points) {
            System.out.printf("(%.3f,%.3f)\n", point.getX(), point.getY());

        }

    }
}

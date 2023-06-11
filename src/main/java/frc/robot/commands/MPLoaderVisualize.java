package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory;
import org.frcteam3539.CTRE_Swerve_Lib.control.Path.State;

public class MPLoaderVisualize {
    // Fill a command sequence from .txt file
    // (recommended for testing only)
    static BufferedWriter writer;

    public static void main(String[] args) {
        try {
            writer = new BufferedWriter(new FileWriter("C:\\Users\\camco\\Desktop\\mp1Out.txt", false));

            MPLoader loader = new MPLoader("C:\\Users\\camco\\Desktop\\mp1.txt", true, true);

            for (Trajectory trajectory : loader.getTrajectories()) {
                for (double i = 0; i < trajectory.getDuration(); i += 0.02) {

                    State s = trajectory.calculate(i).getPathState();
                    writer.append("\n");
                    writer.append("(" + s.getPose2d().getX() + "," + s.getPose2d().getY() + ")");
                }
            }
            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
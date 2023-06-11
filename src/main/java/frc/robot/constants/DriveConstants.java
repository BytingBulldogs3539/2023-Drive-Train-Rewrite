// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

/** Add your docs here. */
public class DriveConstants extends BBConstants {
	public DriveConstants() {
		super("/home/lvuser/DriveConstants.ini", true);
		save();
	}

	public static double wheelTrackWidth = 0.5969; // In meters
	public static double wheelTrackLength = 0.5969; // In meters
	public static double wheelDiameter = 0.1016; // In meters
	public static double driveGearReduction = 6.55; // driving gear/driven gear
	public static double steerGearReduction = 10.286; // driving gear/driven gear
	public static double swerveModuleSlipCurrent = 400;

	public static double FLSteerOffset = -.822754;
	public static double FRSteerOffset = -.711914;
	public static double BLSteerOffset = -.380127;
	public static double BRSteerOffset = -.149902;

	public static double TranslationkP = 20.0;
	public static double TranslationkI = 0.0;
	public static double TranslationkD = 0.0;

	public static double TranslationkV = 1.0;
	public static double TranslationkA = 0.0;
	public static double TranslationkS = 0.0;

	public static double RotationkP = 0.0;
	public static double RotationkI = 0.0;
	public static double RotationkD = 0.0;
	public static double RotationkF = 0.0;

	public static double steerkP = 30.0;
	public static double steerkI = 0.0;
	public static double steerkD = 0.2;

	public static double drivekP = 8.0;
	public static double drivekI = 0.8;
	public static double drivekD = 0.008;

	public static double driveSpeedMultiplier = 0.5;
	public static double rotationSpeedMultiplier = 0.2;
}

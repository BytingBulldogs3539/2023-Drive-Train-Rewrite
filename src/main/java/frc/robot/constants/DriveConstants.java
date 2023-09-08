package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

public class DriveConstants extends BBConstants{
	public DriveConstants() {
		super("/home/lvuser/DriveConstants.ini", true);
		save();
	}
	public static double wheelTrackWidth = 0.5969;
	public static double wheelTrackLength = 0.5969;
	public static double wheelDiameter = 0.1016;
	public static double driveGearReduction = 6.55;
	public static double steerGearReduction = 10.286;
	public static double swerveModuleSlipCurrent = 400.0;
	public static double FLSteerOffset = -0.459473;
	public static double FRSteerOffset = -0.625244;
	public static double BLSteerOffset = -0.442139;
	public static double BRSteerOffset = -0.252197;
	public static double TranslationkP = 20.0;
	public static double TranslationkI = 0.0;
	public static double TranslationkD = 0.0;
	public static double TranslationkV = 1.0;
	public static double TranslationkA = 0.0;
	public static double TranslationkS = 0.0;
	public static double RotationkP = 10.0;
	public static double RotationkI = 0.0;
	public static double RotationkD = 0.7;
	public static double RotationkF = 0.0;
	public static double steerkP = 30.0;
	public static double steerkI = 0.0;
	public static double steerkD = 0.0;
	public static double drivekP = 8.0;
	public static double drivekI = 0.0;
	public static double drivekD = 0.008;
	public static double driveSpeedMultiplier = 0.5;
	public static double rotationSpeedMultiplier = 0.2;
	public static double turboRotationSpeedMultiplier = 0.25;
	public static double slowSpeedMultiplier = 0.25;
	public static double slowRotationSpeedMultiplier = 0.125;
}

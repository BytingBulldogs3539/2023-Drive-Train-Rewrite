package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

public class ElevatorConstants extends BBConstants{
	public ElevatorConstants() {
		super("/home/lvuser/ElevatorConstants.ini", true);
		save();
	}
	public static int ledCount = 100;
	public static double maxArmVelocity = 400.0;
	public static double maxArmAcceleration = 400.0;
	public static double maxArmRotationVelocity = 600.0;
	public static double maxArmRotationAcceleration = 600.0;
	public static double ElevatorConversionRatio = 0.0039814506849375;
	public static double ElevatorRotationMagnetOffset = -48.867;
	public static double ElevatorKp = 0.1;
	public static double ElevatorKi = 0.0;
	public static double ElevatorKd = 0.0;
	public static double ElevatorRotationKp = 0.02;
	public static double ElevatorRotationKi = 0.001;
	public static double ElevatorRotationKd = 0.0;
	public static double ElevatorMinExtension = 78.74;
	public static double ElevatorMaxExtension = 157.4;
	public static double WristRotationMagnetOffset = 117.07;
	public static double WristKp = 50.0;
	public static double WristKi = 0.0;
	public static double WristKd = 0.0;
	public static double IntakeLimitMin = -90.0;
	public static double IntakeLimitMax = -45.0;
	public static int wristSoftMax = 180;
	public static int wristSoftMin = 0;
	public static double ElevatorRotationFeedforwardRatio = 0.001;
	public static int elevatorSoftMax = 78;
	public static int elevatorSoftMin = 0;
	public static int elevatorRotationSoftMax = -780;
	public static int elevatorRotationSoftMin = 2250;
	public static double frontCubeHighX = 139.4;
	public static double frontCubeHighY = 42.1;
	public static double frontCubeMidX = 100.0;
	public static double frontCubeMidY = 8.0;
	public static double frontCubeLowX = 62.4;
	public static double frontCubeLowY = -63.2;
	public static double frontCubeIntakeX = 27.0;
	public static double frontCubeIntakeY = -74.1;
	public static double backCubeHighX = -139.3;
	public static double backCubeHighY = 42.1;
	public static double backCubeMidX = -100.0;
	public static double backCubeMidY = -3.0;
	public static double backCubeLowX = -62.4;
	public static double backCubeLowY = -63.2;
	public static double backCubeIntakeX = -74.2;
	public static double backCubeIntakeY = -77.7;
	public static double frontConeHighX = 142.3;
	public static double frontConeHighY = 42.5;
	public static double frontConeMidX = 103.2;
	public static double frontConeMidY = 11.1;
	public static double frontConeLowX = 77.5;
	public static double frontConeLowY = -45.9;
	public static double frontConeIntakeX = 64.3;
	public static double frontConeIntakeY = -60.9;
	public static double frontConeGroundX = 31.0;
	public static double frontConeGroundY = -93.5;
	public static double frontCubeLowIntakeX = 58.88;
	public static double frontCubeLowIntakeY = -75.88;
	public static double backConeHighX = -142.3;
	public static double backConeHighY = 35.9;
	public static double backConeMidX = -103.2;
	public static double backConeMidY = 11.1;
	public static double backConeLowX = -73.5;
	public static double backConeLowY = -49.9;
	public static double backConeIntakeX = -72.3;
	public static double backConeIntakeY = -57.1;
	public static double backConeHumanPlayerX = -83.1;
	public static double backConeHumanPlayerY = 17.0;
	public static double backCubeHumanPlayerX = -83.1;
	public static double backCubeHumanPlayerY = 35.0;
	public static double frontConeHumanPlayerX = 83.1;
	public static double frontConeHumanPlayerY = 17.0;
	public static double frontCubeHumanPlayerX = 83.1;
	public static double frontCubeHumanPlayerY = 35.0;
}

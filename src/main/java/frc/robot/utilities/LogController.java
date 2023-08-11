package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.LogFileUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;

import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class LogController {
    //  +-----------------------------------+
    //  |  SET ROBOT LOGGING SETTINGS HERE  |
    //  +-----------------------------------+

    static boolean USE_LOGGING = true;

    static boolean LOG_DRIVE_SUBSYSTEM = true;
    static boolean LOG_ARM_SUBSYSTEM = true;
    static boolean LOG_INTAKE_SUBSYSTEM = true;
    static boolean LOG_LEDS = true;

    static boolean SAVE_TO_FILE = false;
    static String LOG_FILE_PATH = "/media/sda1/";
    static boolean USE_NETWORK_TABLES = true;

    //  +---------------------------------+
    //  |  END OF ROBOT LOGGING SETTINGS  |
    //  +---------------------------------+

    DriveSubsystem driveSubsystem;
    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    LEDSubsystem leds;

    public LogController(
        DriveSubsystem drive,
        ArmSubsystem arm,
        IntakeSubsystem intake,
        LEDSubsystem leds)
    {
        if (!USE_LOGGING) return;
        this.driveSubsystem = drive;
        this.armSubsystem = arm;
        this.intakeSubsystem = intake;
        this.leds = leds;

        Logger logger = Logger.getInstance();
        logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

		if (Robot.isReal()) {
			if (SAVE_TO_FILE) logger.addDataReceiver(new WPILOGWriter(LOG_FILE_PATH));
			if (USE_NETWORK_TABLES) logger.addDataReceiver(new NT4Publisher());
		} else {
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
		}
		
		//logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
		logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    public void logPeriodic() {
        if (!USE_LOGGING) return;
        logDriveSubsystem();
        logArmSubsystem();
        logIntakeSubsystem();
        logLEDs();
    }

    public void logDriveSubsystem() {
        if (!LOG_DRIVE_SUBSYSTEM) return;
        driveSubsystem.log();
    }

    public void logArmSubsystem() {
        if (!LOG_ARM_SUBSYSTEM) return;
        armSubsystem.log();
    }

    public void logIntakeSubsystem() {
        if (!LOG_INTAKE_SUBSYSTEM) return;
        intakeSubsystem.log();
    }

    public void logLEDs() {
        if (!LOG_LEDS) return;
        leds.log();
    }
}

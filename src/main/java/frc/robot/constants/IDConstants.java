// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {

        public IDConstants() {
                super("/home/lvuser/IDConstants.ini", false);
                save();
        }

        public static String swerveCanbusName = "canivore";

        public static int FLDriveID = 16;
        public static int FLCanCoderID = 30;
        public static int FLSteeringID = 15;

        public static int FRDriveID = 14;
        public static int FRCanCoderID = 33;
        public static int FRSteeringID = 13;

        public static int BLDriveID = 0;
        public static int BLCanCoderID = 31;
        public static int BLSteeringID = 1;

        public static int BRDriveID = 2;
        public static int BRCanCoderID = 32;
        public static int BRSteeringID = 3;

        public static int PigeonID = 23;
        
        public static int IntakeMotorID = 17;
        public static String IntakeMotorCanName = "rio";

        public static int groundIntakeRightID = 12;
        public static String groundIntakeRightCanName = "rio";

        public static int groundIntakeLeftID = 19;
        public static String groundIntakeLeftCanName = "rio";

        public static int WristMotorID = 18;

        public static int WristEncoderID = 44;

        public static int ElevatorMotorID = 5;
        public static String ElevatorMotorCanName = "rio";

        public static int ElevatorRotationEncoderID = 43;
        public static String ElevatorRotationEncoderCanName = "rio";

        public static int ElevatorRotationMotorID = 11;
        public static String ElevatorRotationMotorCanName = "rio";

        public static int PDHID = 21;
        public static String PDHCanName = "rio";

        public static int CandleID = 9;
        public static String CandleCanName = "canivore";

        public static int CanifierID = 45;
        public static String CanifierCanName = "";

}

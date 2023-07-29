// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ArmSubsystem.Arm;
import frc.robot.subsystems.ArmSubsystem.Wrist;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class IntakeCommand extends CommandBase {
    /** Creates a new IntakeCommand. */
    double intakeSpeed;
    boolean useSensor = true;
    Debouncer debouncer = new Debouncer(0.1,DebounceType.kBoth);

    Arm initialArm;
    Wrist initialWrist;
    IntakeSubsystem intakeSub;
    ArmSubsystem elevatorSub;
    LEDSubsystem ledSub;

    public IntakeCommand(IntakeSubsystem intakeSub, ArmSubsystem elevatorSub, LEDSubsystem ledSub, double speed) {
        this.intakeSub = intakeSub;
        this.ledSub = ledSub;
        this.elevatorSub = elevatorSub;
        this.intakeSpeed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
    }
    
    public IntakeCommand(IntakeSubsystem intakeSub,ArmSubsystem elevatorSub, LEDSubsystem ledSub, double speed, boolean useSensor) {
        this.intakeSub = intakeSub;
        this.ledSub = ledSub;
        this.elevatorSub = elevatorSub;
        this.intakeSpeed = speed;
        this.useSensor = useSensor;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intakeSub.setIntakeSpeed(intakeSpeed);

        initialArm = elevatorSub.getArmLevel();
        initialWrist = elevatorSub.getWristOrientation();

        // Flash on intake/extake. Must keep track to prevent double intake lights
        // i.e. when briefly intaking a piece you picked up earlier in a different mode
        if (intakeSpeed < 0 && initialWrist == Wrist.cone 
                || intakeSpeed < 0 && initialArm == Arm.groundIntake
                || intakeSpeed > 0 && initialWrist == Wrist.cube) {
            ledSub.intake();
        }
        else ledSub.extake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (useSensor) {
        //     boolean pieceDetected = debouncer.calculate(RobotContainer.elevatorSubsystem.getIntakeSensor());
        //     if (pieceDetected && RobotContainer.elevatorSubsystem.getArmLevel() == Arm.groundIntake) {
        //         RobotContainer.elevatorSubsystem.setArmLevel(Arm.intake);
        //     } 
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSub.setIntakeSpeed(0);
        
        // If we were intaking a piece, set color to that piece
        if (intakeSpeed < 0 && initialWrist == Wrist.cone 
                || intakeSpeed < 0 && initialArm == Arm.groundIntake
                || intakeSpeed > 0 && initialWrist == Wrist.cube) {
            ledSub.pieceTaken(initialArm, initialWrist);
        }
        // If we were placing a piece, set back to default lights
        else ledSub.setLEDs(LEDState.ON);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

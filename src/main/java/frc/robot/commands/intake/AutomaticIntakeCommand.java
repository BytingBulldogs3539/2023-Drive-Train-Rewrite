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

public class AutomaticIntakeCommand extends CommandBase {
    /** Creates a new IntakeCommand. */
    double intakeSpeed = 0.0;
    boolean useSensor = true;
    Debouncer debouncer = new Debouncer(0.1, DebounceType.kBoth);

    Arm initialArm;
    Wrist initialWrist;
    IntakeSubsystem intakeSub;
    ArmSubsystem elevatorSub;
    LEDSubsystem ledSub;

    boolean intake;

    int intakeConeSpeed = -1;
    int extakeConeSpeed = 1;

    int intakeCubeSpeed = 1;
    int extakeCubeSpeed = -1;

    /**
     * 
     * @param intakeSub
     * @param elevatorSub
     * @param ledSub
     * @param intake      true will intake the object the arm is setup to pickup or
     *                    place. false will extake the object the arm is setup to
     *                    pickup or place.
     */
    public AutomaticIntakeCommand(IntakeSubsystem intakeSub, ArmSubsystem elevatorSub, LEDSubsystem ledSub,
            boolean intake) {
        this.intakeSub = intakeSub;
        this.ledSub = ledSub;
        this.elevatorSub = elevatorSub;
        this.intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        initialArm = elevatorSub.getArmLevel();
        initialWrist = elevatorSub.getWristOrientation();

        

        if (intake == true) {
            if (initialWrist == Wrist.cube)
                intakeSpeed = intakeCubeSpeed;
            if (initialWrist == Wrist.cone)
                intakeSpeed = intakeConeSpeed;
            ledSub.intake();
        }
        else {
            if (initialWrist == Wrist.cube)
                intakeSpeed = extakeCubeSpeed;
            if (initialWrist == Wrist.cone)
                intakeSpeed = extakeConeSpeed;
            ledSub.extake();
        }
        

        intakeSub.setIntakeSpeed(intakeSpeed);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSub.setIntakeSpeed(0);

        if(initialWrist==Wrist.cone)
            ledSub.setLEDs(LEDState.CONE);
        else
            ledSub.setLEDs(LEDState.CUBE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

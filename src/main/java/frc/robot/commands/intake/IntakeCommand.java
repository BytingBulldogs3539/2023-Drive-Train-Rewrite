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
        

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

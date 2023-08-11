// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.*;
import frc.robot.utilities.ArmTrajectoryGenerator;
import frc.robot.utilities.ArmTrajectoryFollower;

public class ArmSubsystem extends SubsystemBase {

	public enum Arm {
		high, middle, low, intake, HumanPlayer, groundIntake, cubeLowIntake
	}

	public enum Wrist {
		cube, cone
	}

	public enum Sides {
		front, back
	}

	CANifier canifier;
	TalonFX extensionMotor;
	TalonSRX wrist;
	CANCoder wristEncoder;
	TalonFX rotationMotor;
	CANCoder rotationEncoder;
	PIDController m_eController = new PIDController(ElevatorConstants.ElevatorKp, ElevatorConstants.ElevatorKi, ElevatorConstants.ElevatorKd);
	PIDController m_rController = new PIDController(ElevatorConstants.ElevatorRotationKp, ElevatorConstants.ElevatorRotationKi,
	ElevatorConstants.ElevatorRotationKd);

	Arm armPosition = Arm.intake;
	Sides side = Sides.front;
	private Wrist wristOrrientation = Wrist.cube;

	ArmTrajectoryGenerator trajectoryHandler = new ArmTrajectoryGenerator(ElevatorConstants.maxArmVelocity,
			ElevatorConstants.maxArmAcceleration, ElevatorConstants.maxArmRotationVelocity,
			ElevatorConstants.maxArmRotationAcceleration, ElevatorConstants.ElevatorMinExtension);

	ShuffleboardTab armTab = Shuffleboard.getTab("Elevator");

	// A representation of the arm.
	Mechanism2d realArmMech = new Mechanism2d(ElevatorConstants.ElevatorMaxExtension * 2,
			ElevatorConstants.ElevatorMaxExtension * 2);
	MechanismRoot2d realRoot = realArmMech.getRoot("armJoint", ElevatorConstants.ElevatorMaxExtension,
			ElevatorConstants.ElevatorMaxExtension);
	MechanismLigament2d realExtension;

	Mechanism2d expectedArmMech = new Mechanism2d(ElevatorConstants.ElevatorMaxExtension * 2,
			ElevatorConstants.ElevatorMaxExtension * 2);
	MechanismRoot2d expectedRoot = expectedArmMech.getRoot("armJoint", ElevatorConstants.ElevatorMaxExtension,
			ElevatorConstants.ElevatorMaxExtension);
	MechanismLigament2d expectedExtension;

	ArmTrajectoryFollower follower = new ArmTrajectoryFollower(armTab, this::getTargetPosition, trajectoryHandler,
			this::getArmPose,
			this::setExtensionSpeed, this::setRotationSpeed, m_rController,
			ElevatorConstants.ElevatorRotationFeedforwardRatio, m_eController,
			ElevatorConstants.ElevatorMinExtension, ElevatorConstants.ElevatorMaxExtension,
			Rotation2d.fromDegrees(ElevatorConstants.elevatorRotationSoftMin / 10.0),
			Rotation2d.fromDegrees(ElevatorConstants.elevatorRotationSoftMax / 10.0), this);

	/** Creates a new ElevatorSubsystem. */
	public ArmSubsystem() {

		extensionMotor = new TalonFX(IDConstants.ElevatorMotorID, IDConstants.ElevatorMotorCanName);
		extensionMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
		extensionMotor.setNeutralMode(NeutralMode.Brake);
		extensionMotor.setSelectedSensorPosition(0);
		extensionMotor.setInverted(true);
		extensionMotor.configForwardSoftLimitThreshold((ElevatorConstants.elevatorSoftMax * 10));
		extensionMotor.configReverseSoftLimitThreshold(ElevatorConstants.elevatorSoftMin * 10);
		extensionMotor.configForwardSoftLimitEnable(true);
		extensionMotor.configReverseSoftLimitEnable(true);
		extensionMotor.configSelectedFeedbackCoefficient(ElevatorConstants.ElevatorConversionRatio);
		extensionMotor.config_kP(0, ElevatorConstants.ElevatorKp);
		extensionMotor.config_kI(0, ElevatorConstants.ElevatorKi);
		extensionMotor.config_kD(0, ElevatorConstants.ElevatorKd);

		extensionMotor.configMotionCruiseVelocity(70);
		extensionMotor.configMotionAcceleration(140);
		extensionMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 120, .5));

		// elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
		// LimitSwitchNormal.NormallyOpen);
		// elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
		// LimitSwitchNormal.NormallyOpen);

		rotationEncoder = new CANCoder(IDConstants.ElevatorRotationEncoderID);
		rotationEncoder.configMagnetOffset(ElevatorConstants.ElevatorRotationMagnetOffset);
		rotationEncoder.configSensorDirection(false);
		rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		rotationEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

		rotationMotor = new TalonFX(IDConstants.ElevatorRotationMotorID,
				IDConstants.ElevatorRotationMotorCanName);
		rotationMotor.configRemoteFeedbackFilter(rotationEncoder, 0);
		rotationMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
		rotationMotor.setNeutralMode(NeutralMode.Brake);
		rotationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
		rotationMotor.configForwardSoftLimitThreshold(ElevatorConstants.elevatorRotationSoftMin);
		rotationMotor.configReverseSoftLimitThreshold(ElevatorConstants.elevatorRotationSoftMax);
		rotationMotor.configForwardSoftLimitEnable(true);
		rotationMotor.configReverseSoftLimitEnable(true);
		rotationMotor.setInverted(true);
		rotationMotor.setSensorPhase(true);
		rotationMotor.configSelectedFeedbackCoefficient(0.87890625); // Constant that defines .1degree
																		// resolution by
																		// multiplying encoder ticks to get
																		// angle from
																		// the cancoder then multiplies by 10
		rotationMotor.configMotionCruiseVelocity(10);
		rotationMotor.configMotionAcceleration(100);
		rotationMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 120, .5));

		rotationEncoder.setPosition(rotationEncoder.getAbsolutePosition());

		wristEncoder = new CANCoder(IDConstants.WristEncoderID);
		wristEncoder.configMagnetOffset(ElevatorConstants.WristRotationMagnetOffset);
		wristEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

		wrist = new TalonSRX(IDConstants.WristMotorID);
		wrist.setNeutralMode(NeutralMode.Brake);
		wrist.configRemoteFeedbackFilter(wristEncoder, 0);
		wrist.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
		wrist.config_kP(0, ElevatorConstants.WristKp);
		wrist.config_kI(0, ElevatorConstants.WristKi);
		wrist.config_kD(0, ElevatorConstants.WristKd);
		wrist.configSelectedFeedbackCoefficient(0.087890625);
		wrist.configMotionAcceleration(250);
		wrist.configMotionCruiseVelocity(500);
		wristEncoder.setPosition(wristEncoder.getAbsolutePosition());

		// armTab.addNumber("Arm X", () -> {
		// 	return this.getArmPose().getX();
		// });
		// armTab.addNumber("Arm Y", () -> {
		// 	return this.getArmPose().getY();
		// });
		// armTab.addNumber("Arm Angle", () -> {
		// 	return getElevatorRotationAngle().getDegrees();
		// });

		// armTab.add("Extension Motor Speed Output", extensionMotor.getMotorOutputPercent());
		// armTab.add("Arm Rotation Motor Speed Output", rotationMotor.getMotorOutputPercent());


		wrist.set(ControlMode.Position, wrist.getSelectedSensorPosition());

		canifier = new CANifier(IDConstants.CanifierID);

		setDefaultCommand(follower);

		realExtension = realRoot.append(
				new MechanismLigament2d("RealExtension", ElevatorConstants.ElevatorMinExtension, 0));
		expectedExtension = expectedRoot.append(
				new MechanismLigament2d("ExpectedExtension", ElevatorConstants.ElevatorMinExtension, 0));

	}
	public void zeroArmOffset()
	{
		rotationEncoder.configMagnetOffset(0);

	}
	public void saveArmRotationOffset()
	{
		ElevatorConstants.ElevatorRotationMagnetOffset = -rotationEncoder.getAbsolutePosition();
		RobotContainer.elevatorConstants.save();
		rotationEncoder.configMagnetOffset(ElevatorConstants.ElevatorRotationMagnetOffset);
	}

	public void zeroWristOffset()
	{
		wristEncoder.configMagnetOffset(0);

	}
	public void saveWristRotationOffset()
	{
		ElevatorConstants.WristRotationMagnetOffset = -wristEncoder.getAbsolutePosition();
		RobotContainer.elevatorConstants.save();
		wristEncoder.configMagnetOffset(ElevatorConstants.WristRotationMagnetOffset);
	}

	public boolean getIntakeSensor() {
		return canifier.getGeneralInput(GeneralPin.LIMF);
	}

	public void setWristOrientation(Wrist orientation) {
		if (getElevatorRotationAngle().getDegrees() > ElevatorConstants.IntakeLimitMax) {
			this.wristOrrientation = orientation;
		}
	}

	public void setWristOrientationOverride(Wrist orientation) {
		this.wristOrrientation = orientation;
	}

	public void setSide(Sides side) {
		this.side = side;
	}

	public Wrist getWristOrientation() {
		return wristOrrientation;
	}

	public Sides getSide() {
		return side;
	}

	public void setArmLevel(Arm arm) {
		this.armPosition = arm;
	}

	public Arm getArmLevel() {
		return armPosition;
	}

	public double getElevatorLength() {
		return (extensionMotor.getSelectedSensorPosition() / 10.0) + ElevatorConstants.ElevatorMinExtension;
	}

	public Rotation2d getElevatorRotationAngle() {
		return Rotation2d.fromDegrees(rotationEncoder.getPosition());
	}

	public Translation2d getArmPose() {

		return new Translation2d(getElevatorLength(), getElevatorRotationAngle());
	}

	public void setExtensionSpeed(double speed) {
		//extensionMotor.set(ControlMode.PercentOutput, 0);
		extensionMotor.set(ControlMode.PercentOutput, speed);
	}

	public void setRotationSpeed(double speed) {
		if (speed > .55) {
			speed = .55;
		}
		if (speed < -.55) {
			speed = -.55;
		}

		//rotationMotor.set(ControlMode.PercentOutput, 0);
		rotationMotor.set(ControlMode.PercentOutput, speed);
	}

	public void setWristBreakMode(boolean enabled) {
		if (enabled) {
			wrist.setNeutralMode(NeutralMode.Brake);
		} else {
			wrist.setNeutralMode(NeutralMode.Coast);
		}
	}

	public void setBreakMode(boolean enabled) {
		if (enabled) {
			rotationMotor.setNeutralMode(NeutralMode.Brake);
			extensionMotor.setNeutralMode(NeutralMode.Brake);
		} else {
			rotationMotor.setNeutralMode(NeutralMode.Coast);
			extensionMotor.setNeutralMode(NeutralMode.Coast);
		}
	}

	public Translation2d getTargetPosition() {
		Translation2d pos = new Translation2d();
		if (side == Sides.front) {
			if (wristOrrientation == Wrist.cone) {
				if (armPosition == Arm.intake) {
					pos = new Translation2d(ElevatorConstants.frontConeIntakeX, ElevatorConstants.frontConeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Translation2d(ElevatorConstants.frontConeLowX, ElevatorConstants.frontConeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Translation2d(ElevatorConstants.frontConeMidX, ElevatorConstants.frontConeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Translation2d(ElevatorConstants.frontConeHighX, ElevatorConstants.frontConeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Translation2d(ElevatorConstants.frontConeHumanPlayerX,
							ElevatorConstants.frontConeHumanPlayerY);
				} else if (armPosition == Arm.cubeLowIntake) {
					pos = new Translation2d(ElevatorConstants.frontConeIntakeX, ElevatorConstants.frontConeIntakeY);
				} else if (armPosition == Arm.groundIntake) {
					pos = new Translation2d(ElevatorConstants.frontConeIntakeX, ElevatorConstants.frontConeIntakeY);
				}
			} else if (wristOrrientation == Wrist.cube) {
				if (armPosition == Arm.intake) {
					pos = new Translation2d(ElevatorConstants.frontCubeIntakeX, ElevatorConstants.frontCubeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Translation2d(ElevatorConstants.frontCubeLowX, ElevatorConstants.frontCubeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Translation2d(ElevatorConstants.frontCubeMidX, ElevatorConstants.frontCubeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Translation2d(ElevatorConstants.frontCubeHighX, ElevatorConstants.frontCubeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Translation2d(ElevatorConstants.frontCubeHumanPlayerX,
							ElevatorConstants.frontCubeHumanPlayerY);
				} else if (armPosition == Arm.groundIntake) {
					pos = new Translation2d(ElevatorConstants.frontConeGroundX,
							ElevatorConstants.frontConeGroundY);
				} else if (armPosition == Arm.cubeLowIntake) {
					pos = new Translation2d(ElevatorConstants.frontCubeLowIntakeX,
							ElevatorConstants.frontCubeLowIntakeY);
				}
			}
		} else if (side == Sides.back) {
			if (wristOrrientation == Wrist.cone) {
				if (armPosition == Arm.intake) {
					pos = new Translation2d(ElevatorConstants.backConeIntakeX, ElevatorConstants.backConeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Translation2d(ElevatorConstants.backConeLowX, ElevatorConstants.backConeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Translation2d(ElevatorConstants.backConeMidX, ElevatorConstants.backConeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Translation2d(ElevatorConstants.backConeHighX, ElevatorConstants.backConeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Translation2d(ElevatorConstants.backConeHumanPlayerX,
							ElevatorConstants.backConeHumanPlayerY);
				} else if (armPosition == Arm.groundIntake) {
					pos = new Translation2d(ElevatorConstants.backConeIntakeX, ElevatorConstants.backConeIntakeY);

				} else if (armPosition == Arm.cubeLowIntake) {
					pos = new Translation2d(ElevatorConstants.backConeIntakeX, ElevatorConstants.backConeIntakeY);

				}
			} else if (wristOrrientation == Wrist.cube) {
				if (armPosition == Arm.intake) {
					pos = new Translation2d(ElevatorConstants.backCubeIntakeX, ElevatorConstants.backCubeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Translation2d(ElevatorConstants.backCubeLowX, ElevatorConstants.backCubeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Translation2d(ElevatorConstants.backCubeMidX, ElevatorConstants.backCubeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Translation2d(ElevatorConstants.backCubeHighX, ElevatorConstants.backCubeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Translation2d(ElevatorConstants.backCubeHumanPlayerX,
							ElevatorConstants.backCubeHumanPlayerY);
				} else if (armPosition == Arm.cubeLowIntake) {
					pos = new Translation2d(ElevatorConstants.backCubeIntakeX, ElevatorConstants.backCubeIntakeY);
				} else if (armPosition == Arm.groundIntake) {
					pos = new Translation2d(ElevatorConstants.backCubeIntakeX, ElevatorConstants.backCubeIntakeY);
				}
			}
		}
		if (pos.getX() == 0 && pos.getY() == 0) {
			pos = new Translation2d(ElevatorConstants.frontConeIntakeX, ElevatorConstants.frontConeIntakeY);
		}
		return pos;
	}

	public void log() {
		realExtension.setAngle(getElevatorRotationAngle());
		realExtension.setLength(getElevatorLength());
		expectedExtension.setAngle(follower.targetTranslation.getAngle());
		expectedExtension.setLength(follower.targetTranslation.getNorm());

		// Logger.getInstance().recordOutput("/Arm/realArmMech", realArmMech);
		// Logger.getInstance().recordOutput("/Arm/expectedArmMech", expectedArmMech);

	}

	@Override
	public void periodic() {
		
		//SmartDashboard.putBoolean("Cube Mode", (wristOrrientation == Wrist.cube) ? true : false);
		// SmartDashboard.putData("RealArm", realArmMech);
		// SmartDashboard.putData("ExpectedArm", expectedArmMech);

		if (getElevatorRotationAngle().getDegrees() > ElevatorConstants.IntakeLimitMax) {
			if (wristOrrientation == Wrist.cube && side == Sides.front) {
				wrist.set(ControlMode.MotionMagic, 0);
			}
			if (wristOrrientation == Wrist.cube && side == Sides.back) {
				wrist.set(ControlMode.MotionMagic, 180);
			}
			if (wristOrrientation == Wrist.cone && side == Sides.back) {
				wrist.set(ControlMode.MotionMagic, 0);
			}
			if (wristOrrientation == Wrist.cone && side == Sides.front) {
				wrist.set(ControlMode.MotionMagic, 180);
			}

		}

		//log();

		 //wrist.set(ControlMode.PercentOutput, 0);
	}
}

package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frcteam3539.CTRE_Swerve_Lib.control.MultiTrajectory;
import org.frcteam3539.CTRE_Swerve_Lib.control.Path.State;

import java.awt.geom.Point2D;

public class ArmTrajectoryFollower extends CommandBase {
	private final Timer m_timer = new Timer();

	State lastState = null;

	private MultiTrajectory m_trajectory;
	private final Supplier<Translation2d> endPointSupplier;
	private final ArmTrajectoryGenerator generator;
	private final Supplier<Translation2d> m_pose;
	private final Supplier<Translation2d> xyPose;
	private final Consumer<Double> setExtenionSpeed;
	private final Consumer<Double> setRotationSpeed;
	private final PIDController m_rController;
	private final PIDController m_eController;
	private final double xKf;
	private double minArmLength = 0.0;
	private double lastTime = 0.0;
	private double maxArmLength = 100.0;
	private Rotation2d minArmRotation = Rotation2d.fromDegrees(0);
	private Rotation2d maxArmRotation = Rotation2d.fromDegrees(0);
	private Translation2d lastEndpoint = new Translation2d();
	ShuffleboardTab tab;

	public ArmTrajectoryFollower(ShuffleboardTab tab, Supplier<Translation2d> endPoint, ArmTrajectoryGenerator generator,
			Supplier<Translation2d> pose, Supplier<Translation2d> xyPose,
			Consumer<Double> setExtenionSpeed,
			Consumer<Double> setRotationSpeed, PIDController rController, double rKf,
			PIDController eController, double minArmLength, double maxArmLength, Rotation2d minArmRotation,
			Rotation2d maxArmRotation, Subsystem... requirements) {
		this.endPointSupplier = (Supplier<Translation2d>) ErrorMessages.requireNonNullParam(endPoint, "endPoint",
				"SwerveControllerCommand");
		this.generator = (ArmTrajectoryGenerator) ErrorMessages.requireNonNullParam(generator, "generator",
				"SwerveControllerCommand");
		this.m_pose = (Supplier<Translation2d>) ErrorMessages.requireNonNullParam(pose, "pose",
				"SwerveControllerCommand");
		this.xyPose = (Supplier<Translation2d>) ErrorMessages.requireNonNullParam(xyPose, "xyPose",
				"SwerveControllerCommand");
		this.m_rController = (PIDController) ErrorMessages.requireNonNullParam(rController, "xController",
				"SwerveControllerCommand");
		this.xKf = (double) ErrorMessages.requireNonNullParam(rKf, "xKf",
				"SwerveControllerCommand");
		this.m_eController = (PIDController) ErrorMessages.requireNonNullParam(eController, "xController",
				"SwerveControllerCommand");
		this.setExtenionSpeed = (Consumer<Double>) ErrorMessages.requireNonNullParam(setExtenionSpeed,
				"setExtensionSpeed",
				"SwerveControllerCommand");
		this.setRotationSpeed = (Consumer<Double>) ErrorMessages.requireNonNullParam(setRotationSpeed,
				"setRotationSpeed",
				"SwerveControllerCommand");

		this.tab = (ShuffleboardTab) ErrorMessages.requireNonNullParam(tab,
				"tab",
				"SwerveControllerCommand");

		this.minArmLength = minArmLength;
		this.maxArmLength = maxArmLength;
		this.minArmRotation = minArmRotation;
		this.maxArmRotation = maxArmRotation;

		addRequirements(requirements);
	}

	public void initialize() {
		this.m_timer.reset();
		this.m_timer.start();
		m_trajectory = generator.generateTrajectories(xyPose.get(), endPointSupplier.get());
		lastState = m_trajectory.calculate(0).getPathState();

	}

	public void execute() {
		// System.out.println(endPointSupplier.get().x+" " + endPointSupplier.get().y);
		if (endPointSupplier.get().getX()!= lastEndpoint.getX() || endPointSupplier.get().getY() != lastEndpoint.getY()) {
			// System.out.println(xyPose.get().x+" " +xyPose.get().y);
			// System.out.println(endPointSupplier.get().x+" " + endPointSupplier.get().y);
			double endPointx = Math.floor(endPointSupplier.get().getX()* 100) / 100.0;
			double endPointy = Math.floor(endPointSupplier.get().getY() * 100) / 100.0;
			double startPointx = Math.floor(xyPose.get().getX() * 100) / 100.0;
			double startPointy = Math.floor(xyPose.get().getY() * 100) / 100.0;
			m_trajectory = generator.generateTrajectories(new Translation2d(startPointx, startPointy),
					new Translation2d(endPointx, endPointy));

			// System.out.println(m_trajectory.getDuration());
			this.m_timer.stop();
			this.m_timer.reset();
			this.m_timer.start();
			lastState = m_trajectory.calculate(0).getPathState();
			// System.out.println("yolo");
			// System.out.println(endPointSupplier.get().x +" : "+endPointSupplier.get().y);
		}

		double curTime = this.m_timer.get();
		SmartDashboard.putNumber("CurTime", curTime);

		State s = this.m_trajectory.calculate(curTime).getPathState();

		Translation2d p = new Translation2d(s.getPose2d().getX(), s.getPose2d().getY());

		if (p.getAngle().getDegrees() < -95) {
			p = new Translation2d(p.getNorm(),p.getAngle().plus(Rotation2d.fromDegrees(360)));
		}

		if (p.getNorm() < ElevatorConstants.ElevatorMinExtension) {
			p = new Translation2d(minArmLength, p.getAngle());
		}

		// if (p.getExtension() < minArmLength) {
		// p = new ArmPosition(p.getRotation(), minArmLength);
		// }
		// if (p.getExtension() > maxArmLength) {
		// p = new ArmPosition(p.getRotation(), maxArmLength);
		// }
		// if (p.getRotation().getDegrees() > maxArmRotation.getDegrees()) {
		// p = new ArmPosition(maxArmRotation, p.getExtension());
		// }
		// if (p.getRotation().getDegrees() < minArmRotation.getDegrees()) {
		// p = new ArmPosition(minArmRotation, p.getExtension());
		// }

		// TODO: add limiting box.

		SmartDashboard.putNumber("Expected Arm X", s.getPose2d().getX());
		SmartDashboard.putNumber("Expected Arm Y", s.getPose2d().getY());

		SmartDashboard.putNumber("Expected Extension", p.getNorm());
		SmartDashboard.putNumber("Expected Rotation", p.getAngle().getDegrees());

		SmartDashboard.putNumber("Real Extension", this.m_pose.get().getNorm());
		SmartDashboard.putNumber("Real Rotation", this.m_pose.get().getAngle().getDegrees());

		SmartDashboard.putNumber("Real Arm x", xyPose.get().getX());
		SmartDashboard.putNumber("Real Arm y", xyPose.get().getY());

		double targetE = this.m_eController.calculate(this.m_pose.get().getNorm(), p.getNorm());
		double targetR = this.m_rController
				.calculate(this.m_pose.get().getAngle().getDegrees(), p.getAngle().getDegrees())
				+ (xKf * s.getPose2d().getX());//+velocityFeedForward;

		setExtenionSpeed.accept(targetE);
		setRotationSpeed.accept(targetR);

		lastEndpoint = endPointSupplier.get();
		lastState = s;
		lastTime = curTime;
	}

	public void end(boolean interrupted) {
		this.m_timer.stop();
		setRotationSpeed.accept(0.0);
		setExtenionSpeed.accept(0.0);
	}

	public boolean isFinished() {
		return false;
	}
}
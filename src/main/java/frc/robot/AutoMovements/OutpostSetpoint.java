package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;


public class OutpostSetpoint {
	/** 10 inches in meters */
	private static final double RETRACT_DISTANCE = 0.35;

	public enum State {
		omwToOutpost,
		atOutpost
	}

	private final LocalizationSubsystem localization;
	private final SwerveSubsystem swerve;
	private final IntakePosition intakePosition;
	private final intaker intakeRoller;
	private State state = State.atOutpost;

	public OutpostSetpoint(LocalizationSubsystem localization, SwerveSubsystem swerve,
			IntakePosition intakePosition, intaker intakeRoller) {
		this.localization = localization;
		this.swerve = swerve;
		this.intakePosition = intakePosition;
		this.intakeRoller = intakeRoller;
	}

	public Pose2d getAllianceSetpoint() {
		return FmsSubsystem.isRedAlliance() ? FieldPoints.getOutpostRed() : FieldPoints.getOutpostBlue();
	}

	public void setRedSetpoint(Pose2d pose) { FieldPoints.setOutpostRed(pose); }
	// Blue outpost is auto-mirrored from red — set red to change blue

	public State getState() {
		return state;
	}

	public Command travelToOutpost() {
		DriveToPose driveToPose = new DriveToPose(swerve, localization, this::getAllianceSetpoint);
		boolean[] intakeDeployed = {false};
		boolean[] intakeRetracted = {false};

		return Commands.runOnce(() -> {
					state = State.omwToOutpost;
					intakeDeployed[0] = false;
					intakeRetracted[0] = false;
				})
				.andThen(
						driveToPose.alongWith(
								Commands.run(() -> {
									// Deploy intake once we enter the X (ALL) phase
									if (!intakeDeployed[0] && driveToPose.isInAllPhase()) {
										intakePosition.deploy();
										intakeRoller.intake();
										intakeDeployed[0] = true;
									}
									// Retract when within 10 inches of target
									if (intakeDeployed[0] && !intakeRetracted[0]
											&& driveToPose.getDistanceToTarget() < RETRACT_DISTANCE) {
										intakePosition.retract();
										intakeRoller.stop();
										intakeRetracted[0] = true;
									}
								})
						)
						.withName("omwToOutpost"))
				.andThen(
						Commands.runOnce(() -> {
							state = State.atOutpost;
						}))
				// Ensure intake is retracted and stopped if interrupted
				.finallyDo((interrupted) -> {
					intakePosition.retract();
					intakeRoller.stop();
				})
				.withName("travelToOutpost");
	}
}

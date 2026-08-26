package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.localization.LocalizationSubsystem;


public class EnterNeutralZone {

	public enum State {
		omwtoNeutralZone_red,
		omwtoNeutralZone_blue,
		atNeutralZone
	}

	private final LocalizationSubsystem localization;
	private final Trailblazer trailblazer;
	private State state = State.atNeutralZone;


	public EnterNeutralZone(LocalizationSubsystem localization, Trailblazer trailblazer) {
		this.localization = localization;
		this.trailblazer = trailblazer;
	}

	public State getState() {
		return state;
	}

		public void setRA1(Pose2d pose) { FieldPoints.setRA1(pose); }
		public void setRB1(Pose2d pose) { FieldPoints.setRB1(pose); }
		public void setRA2(Pose2d pose) { FieldPoints.setRA2(pose); }
		public void setRB2(Pose2d pose) { FieldPoints.setRB2(pose); }

		// Blue neutral zone poses are auto-mirrored from red — set red to change blue


		public Command omwtoNeutralZone_red() {
			var constraints = new AutoConstraintOptions();

			var segR1_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getRA1()));
			var segR1_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getRA1()), new AutoPoint(FieldPoints.getRB1()));
			var series1 = trailblazer.followSegment(segR1_1, true)
					.andThen(trailblazer.followSegment(segR1_2, true))
					.withName("omwtoNeutralZone_red_series1");

			var segR2_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getRA2()));
			var segR2_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getRA2()), new AutoPoint(FieldPoints.getRB2()));
			var series2 = trailblazer.followSegment(segR2_1, true)
					.andThen(trailblazer.followSegment(segR2_2, true))
					.withName("omwtoNeutralZone_red_series2");

			var chooseSeries1 = (java.util.function.BooleanSupplier) () -> {
				var current = localization.getPose();
				return current.getTranslation().getDistance(FieldPoints.getRA1().getTranslation())
					<= current.getTranslation().getDistance(FieldPoints.getRA2().getTranslation());
			};

			return Commands.runOnce(() -> {
						CommandScheduler.getInstance().cancelAll();
						state = State.omwtoNeutralZone_red;
					})
					.andThen(Commands.either(series1, series2, chooseSeries1))
					.andThen(Commands.runOnce(() -> state = State.atNeutralZone))
					.withName("EnterNeutralZone_red");
		}


		public Command omwtoNeutralZone_blue() {
			var constraints = new AutoConstraintOptions();

			var segB1_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getBA1()));
			var segB1_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getBA1()), new AutoPoint(FieldPoints.getBB1()));
			var series1 = trailblazer.followSegment(segB1_1, true)
					.andThen(trailblazer.followSegment(segB1_2, true))
					.withName("omwtoNeutralZone_blue_series1");

			var segB2_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getBA2()));
			var segB2_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getBA2()), new AutoPoint(FieldPoints.getBB2()));
			var series2 = trailblazer.followSegment(segB2_1, true)
					.andThen(trailblazer.followSegment(segB2_2, true))
					.withName("omwtoNeutralZone_blue_series2");

			var chooseSeries1 = (java.util.function.BooleanSupplier) () -> {
				var current = localization.getPose();
				return current.getTranslation().getDistance(FieldPoints.getBA1().getTranslation())
					<= current.getTranslation().getDistance(FieldPoints.getBA2().getTranslation());
			};

			return Commands.runOnce(() -> {
						CommandScheduler.getInstance().cancelAll();
						state = State.omwtoNeutralZone_blue;
					})
					.andThen(Commands.either(series1, series2, chooseSeries1))
					.andThen(Commands.runOnce(() -> state = State.atNeutralZone))
					.withName("EnterNeutralZone_blue");
		}
}

package frc.robot.autos;

import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.FlywheelSubsystem.Drum;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.FlywheelSubsystem.DrumStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.Intake.BeamBreak;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import org.wpilib.math.filter.Debouncer;
import frc.robot.imu.BumpCrossingTracker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.lib.BLine.FollowPath;
import java.util.Map;

/**
 * All point-to-point autonomous routines live here.
 *
 * ===== HOW TO ADD A NEW AUTO =====
 * 1. Add a new method below (copy an existing one as a template)
 * 2. Use AutoRoutine.create(swerve, localization, pathBuilder, bumpCrossingTracker) to start building
 * 3. Chain steps:
 *      .startAt(x, y, deg)            - set starting pose
 *      .driveTo(x, y, deg)            - drive to a field pose
 *      .runOnce(() -> ...)             - instant action
 *      .run(command)                   - run command, wait for it to finish
 *      .runFor(seconds, command)       - run command for N seconds
 *      .waitSeconds(seconds)           - pause
 *      .doWhileDriving(command)        - run in parallel with NEXT driveTo
 *      .build()                        - finalize
 * 4. Register in constructor: chooser.addOption("Name", myNewAuto())
 *
 * ===== HOW TO ADD A POINT TO AN EXISTING AUTO =====
 * Just insert .driveTo(x, y, heading) wherever you want in the chain.
 * Add .runOnce() / .doWhileDriving() around it for actions.
 *
 * ===== BLUE AUTOS =====
 * Blue autos are mirrored from red using AutoRoutine.createMirrored().
 * Only define red poses — blue is automatic.
 */
public class PointToPointAutos {
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final FollowPath.Builder pathBuilder;
  private final BumpCrossingTracker bumpCrossingTracker;
  private final Drum drum;
  private final DrumStateMachine drumSM;
  private final HoodStateMachine hoodSM;
  private final HeadingLock headingLock;
  private final LookupTable turretLookup;
  private final Indexer indexer;
  private final Hopper hopper;
  private final intaker intakeRoller;
  private final IntakePosition intakePosition;
  private final BeamBreak beamBreak;

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public PointToPointAutos(
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      FollowPath.Builder pathBuilder,
      BumpCrossingTracker bumpCrossingTracker,
      Drum drum,
      DrumStateMachine drumSM,
      HoodStateMachine hoodSM,
      HeadingLock headingLock,
      LookupTable turretLookup,
      Indexer indexer,
      Hopper hopper,
      intaker intakeRoller,
      IntakePosition intakePosition,
      BeamBreak beamBreak) {
    this.beamBreak = beamBreak;
    this.swerve = swerve;
    this.localization = localization;
    this.pathBuilder = pathBuilder;
    this.bumpCrossingTracker = bumpCrossingTracker;
    this.drum = drum;
    this.drumSM = drumSM;
    this.hoodSM = hoodSM;
    this.headingLock = headingLock;
    this.turretLookup = turretLookup;
    this.indexer = indexer;
    this.hopper = hopper;
    this.intakeRoller = intakeRoller;
    this.intakePosition = intakePosition;

    // ===== REGISTER ALL AUTOS HERE =====
    chooser.setDefaultOption("Do Nothing", Commands.none());
    chooser.addOption("Red Right", RedRight());
    chooser.addOption("Red Left", RedLeft());
    chooser.addOption("Blue Left", BlueLeft());
    chooser.addOption("Blue Right", BlueRight());
    chooser.addOption("Blue Mid", BlueMiddle());
    chooser.addOption("Red Mid", RedMiddle());
      chooser.addOption("Red Mid Depot", RedMiddleDepot());
       chooser.addOption("Blue Mid Depot", BlueMiddleDepot());



    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /** Get the currently selected auto c>ommand from the dashboard chooser. */
  public Command getSelected() {
    return chooser.getSelected();
  }

  // =====================================================================
  //  MIRROR HELPERS — shorthand for FieldPoints mirror utilities
  // =====================================================================

  // Mirror helpers now handled by AutoRoutine.createMirrored

  // =====================================================================
  //  HELPER COMMANDS - reusable building blocks for any auto
  // =====================================================================

  private static final double SHOOT_SPEED_THRESHOLD = 0.5;
  private static final int SHOOT_READY_FRAME_THRESHOLD = 2;
  private int autoShootReadyFrames = 0;
  /** Same falling debounce teleop uses, so one bad frame does not stop the feed mid-shot. */
  private final Debouncer autoShootReadyDebouncer =
      new Debouncer(0.08, Debouncer.DebounceType.kFalling);

  /** Offset applied after a right-side pose has been mirrored onto the red-left side. */
  private record PoseTweak(double xMeters, double yMeters, double headingDegrees) {
    private Pose2d apply(Pose2d pose) {
      return new Pose2d(
          pose.getX() + xMeters,
          pose.getY() + yMeters,
          pose.getRotation().plus(Rotation2d.fromDegrees(headingDegrees)));
    }
  }

  private static final PoseTweak NO_TWEAK = new PoseTweak(0.0, 0.0, 0.0);

  /*
   * Optional adjustments for the left-side copy. Keys match the names in SideAuto below.
   * Tweaks use field coordinates after the Y mirror. For example:
   *
   * Map.entry("firstShoot", new PoseTweak(0.05, -0.02, 2.0))
   *
   * A tweak is also alliance-mirrored into Blue Right, keeping both alliance autos consistent.
   */
  private static final Map<String, PoseTweak> LEFT_AUTO_TWEAKS = Map.ofEntries();

  private static Pose2d sidePose(
      String name, double x, double y, double headingDegrees, boolean leftSide) {
    Pose2d rightPose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
    if (!leftSide) {
      return rightPose;
    }
    Pose2d mirrored = FieldPoints.mirrorPoseLeftRight(rightPose);
    return LEFT_AUTO_TWEAKS.getOrDefault(name, NO_TWEAK).apply(mirrored);
  }

  /** Same as teleop right trigger press → hold → release, as a command. */
  private Command startShooting() {
    return Commands.runOnce(() -> {
          turretLookup.enable();
          headingLock.enableForAlliance();
          intakeRoller.feed();
          hopper.feed();
          autoShootReadyFrames = 0;
        })
        .andThen(Commands.run(() -> {
          var params = turretLookup.getParameters();
          // Take the commanded values from the subsystem rather than re-deriving them here.
          // LookupTable already commands the drum and hood every loop, and re-deriving ignores the
          // manual tuning override -- the two then fight each other at 50 Hz.
          drumSM.requestRpm(turretLookup.getActiveRpm());
          hoodSM.requestDegrees(turretLookup.getActiveHoodDeg());

          var speeds = swerve.getRobotRelativeSpeeds();
          double robotSpeed = Math.hypot(speeds.vx, speeds.vy);
          // Heading deliberately excluded, matching teleop: the heading lock keeps aiming, but a
          // snap that settles a couple of degrees short must not stall the shot forever.
          boolean allReadyRaw = params.isValid()
              && robotSpeed < SHOOT_SPEED_THRESHOLD
              && drum.isAtGoal();

          if (autoShootReadyDebouncer.calculate(allReadyRaw)) {
            autoShootReadyFrames++;
          } else {
            autoShootReadyFrames = 0;
          }

          if (autoShootReadyFrames >= SHOOT_READY_FRAME_THRESHOLD) {
            indexer.feed();
            hopper.feed();
            // Feeding: sweep the arm end to end to keep balls moving toward the indexer.
            intakePosition.swing();
          } else {
            indexer.stop();
            hopper.stop();
            // Waiting on the shot gate with nothing at the sensor -- run the staged agitation
            // (20% of travel, then 50%, then all the way down) to work a ball into position.
            if (!beamBreak.hasBall()) {
              intakePosition.pulse();
            } else {
              intakePosition.deploy();
            }
          }
        }))
        .withName("AutoShoot");
  }

  private Command stopShooting() {
    return Commands.runOnce(() -> {
      turretLookup.disable();
      headingLock.disableLock();
      drumSM.requestOff();
      hoodSM.requestOff();
      intakePosition.deploy();
      indexer.stop();
      intakeRoller.stop();
      hopper.stop();
    });
  }

  private Command startIntaking() {
    return Commands.runOnce(() -> {
      intakePosition.deploy();
      intakeRoller.feed();
      hopper.feed();
    });
  }

  private Command stopIntaking() {
    return Commands.runOnce(() -> {
      intakePosition.retract();
      intakeRoller.stop();
      hopper.stop();
    });
  }

  private Command stopAll() {
    return Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
  drumSM.requestOff();
      hoodSM.requestOff();
      indexer.stop();
      hopper.stop();
      intakeRoller.stop();
      intakePosition.retract();
    });
  }

  // =====================================================================
  //  RED AUTOS (source of truth)
  // =====================================================================




  /**
   * Defines one red-right source path. The left-side autos are generated by reflecting every pose
   * across the field width, then applying any named entries from LEFT_AUTO_TWEAKS.
   */
  private Command SideAuto(boolean mirrorAlliance, boolean leftSide) {
    var routine = mirrorAlliance
        ? AutoRoutine.createMirrored(swerve, localization, pathBuilder, bumpCrossingTracker)
        : AutoRoutine.create(swerve, localization, pathBuilder, bumpCrossingTracker);
    Command command = routine
        .startAt(sidePose("start", 12.16, 7.383, 90, leftSide))
        .run(startIntaking()) // don't cut off intaking early if we get stuck on the first waypoint
        .driveToAll(sidePose("firstPickupOuter", 8.88, 7.321, 115, leftSide))
        .driveToAll(sidePose("firstPickupInner", 8.88, 4.471, 115, leftSide))
        .driveToAll(sidePose("lineupToBump", 9.71, 5.67, 270, leftSide))
        // Crossing runs +X in red coords (9.71 -> 13.2). No landing point yet: a pose reset to a
        // guessed location is worse than none, so this is velocity override only until the real
        // landing spot is measured.
        .bumpCross(Rotation2d.kZero)
        .driveToAll(sidePose("firstShootApproach", 13.2, 5.67, 270, leftSide))
        .driveToAll(sidePose("firstShoot", 13.7, 5.2, 195, leftSide))
        .runFor(3, startShooting())
        .run(stopShooting())
        .driveToAll(sidePose("trenchEntry", 13.71, 7.430, 0, leftSide))
        .driveToAll(sidePose("trenchExit", 10.71, 7.430, 0, leftSide))
        .driveToAll(sidePose("secondPickupOuter", 9.51, 6.36, 90, leftSide), 2.7)
        .driveToAll(sidePose("secondPickupInner", 9.38, 3.99, 90, leftSide), 2.7)
        .driveToAll(sidePose("secondPickupHubSlam", 10.58, 4.0, 269, leftSide), 2.7)
        .driveToAll(sidePose("lineupToBump2", 9.71, 5.67, 270, leftSide))
        .bumpCross(Rotation2d.kZero)
        .driveToAll(sidePose("secondShootApproach", 13.2, 5.67, 270, leftSide))
        .driveToAll(sidePose("secondShoot", 13.7, 5.2, 270, leftSide))
        .runFor(3, startShooting())
        .run(stopAll())
        .build();

    String name = mirrorAlliance
        ? (leftSide ? "Blue Right" : "Blue Left")
        : (leftSide ? "Red Left" : "Red Right");
    return command.withName(name);
  }

  private Command MiddleAuto(boolean mirror) {
    var routine = mirror
        ? AutoRoutine.createMirrored(swerve, localization, pathBuilder, bumpCrossingTracker)
        : AutoRoutine.create(swerve, localization, pathBuilder, bumpCrossingTracker);
    return routine
        .startAt(13.02, 4.05, 180)
        .driveToAll(14.9, 4.05, 180)
        .runFor(5, startShooting())
        .run(stopShooting())
        .run(stopAll())
        .build()
        .withName(mirror ? "Blue Mid" : "Red Mid");
  }

  private Command MiddleAutoDepot(boolean mirror) {
    var routine = mirror
        ? AutoRoutine.createMirrored(swerve, localization, pathBuilder, bumpCrossingTracker)
        : AutoRoutine.create(swerve, localization, pathBuilder, bumpCrossingTracker);
    return routine
        .startAt(13.02, 4.05, 180)
        .run(startIntaking()) 
        .driveToAll(14.4, 4.05, 180)
        .runFor(5, startShooting())
        .run(stopShooting())
        .driveToAll(14.73, 2.44, 0)
        .driveToAll(16.01, 2.61, 90)
        .driveToAll(16, 1.28, 90)
        .driveToAll(14.22, 3.2, 160)
        .runFor(5, startShooting())
        .run(stopShooting())
        .run(stopAll())
        .build()
        .withName(mirror ? "Blue Mid Depot" : "Red Mid Depot");
  }











  private Command RedRight() { return SideAuto(false, false); }
  private Command RedLeft() { return SideAuto(false, true); }
  private Command BlueLeft() { return SideAuto(true, false); }
  private Command BlueRight() { return SideAuto(true, true); }
  private Command RedMiddle() { return MiddleAuto(false); }
  private Command BlueMiddle() { return MiddleAuto(true); }
  private Command BlueMiddleDepot() {return MiddleAutoDepot(true);}
  private Command RedMiddleDepot() { return MiddleAutoDepot(false); }
   

}



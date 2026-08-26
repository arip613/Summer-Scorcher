package frc.robot.autos;

import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.FlywheelSubsystem.Drum;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.FlywheelSubsystem.DrumStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
// BLINE DISABLED (BLine-Lib v0.9.1 targets 2026 WPILib):
// import frc.robot.lib.BLine.FollowPath;

/**
 * All point-to-point autonomous routines live here.
 *
 * ===== HOW TO ADD A NEW AUTO =====
 * 1. Add a new method below (copy an existing one as a template)
 * 2. Use AutoRoutine.create(swerve, localization) to start building
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
  // BLINE DISABLED: private final FollowPath.Builder pathBuilder;
  private final Drum drum;
  private final DrumStateMachine drumSM;
  private final HoodStateMachine hoodSM;
  private final HeadingLock headingLock;
  private final LookupTable turretLookup;
  private final Indexer indexer;
  private final Hopper hopper;
  private final intaker intakeRoller;
  private final IntakePosition intakePosition;

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public PointToPointAutos(
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      // BLINE DISABLED: FollowPath.Builder pathBuilder,
      Drum drum,
      DrumStateMachine drumSM,
      HoodStateMachine hoodSM,
      HeadingLock headingLock,
      LookupTable turretLookup,
      Indexer indexer,
      Hopper hopper,
      intaker intakeRoller,
      IntakePosition intakePosition) {
    this.swerve = swerve;
    this.localization = localization;
    // BLINE DISABLED: this.pathBuilder = pathBuilder;
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
    chooser.addOption("Blue Left", BlueRight());
    chooser.addOption("Blue Right", BlueLeft());
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
          drumSM.requestRpm(params.flywheelRpm());
          hoodSM.requestDegrees(Math.toDegrees(params.hoodAngleRad()));

          var speeds = swerve.getRobotRelativeSpeeds();
          double robotSpeed = Math.hypot(speeds.vx, speeds.vy);
          boolean allReady = params.isValid()
              && robotSpeed < SHOOT_SPEED_THRESHOLD
              && drum.isAtGoal()
              && headingLock.isSettled();

          if (allReady) {
            autoShootReadyFrames++;
          } else {
            autoShootReadyFrames = 0;
          }

          if (autoShootReadyFrames >= SHOOT_READY_FRAME_THRESHOLD) {
            indexer.feed();
            hopper.feed();
          } else {
            indexer.stop();
            hopper.stop();
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




private Command LeftAuto(boolean mirror) {
  var routine = mirror
      ? AutoRoutine.createMirrored(swerve, localization)
      : AutoRoutine.create(swerve, localization);
  return routine
      .startAt(12.16, 0.638, 270)
      .run(startIntaking()) // don't cut off intaking early if we get stuck on the first waypoint
    .driveToAll(9.2, 0.70, 245)
    .driveToAll(9.2, 3.55, 245)
    .driveToAll(9.71, 2.27, 10, 2.6)
    .driveToAll(14.7, 2.27, 130, 2.6)
    .driveToAll(15.7, 2.82, 130)
    .runFor(3, startShooting())
    .run(stopShooting())
    .driveToAll(10.71, 2.27, 0)
    .driveToAll(9.51, 1.66, 270, 2.7)
    .driveToAll(9.38, 4.03, 270, 2.7)
    .driveToAll(10.58, 4.02, 130, 2.7)
    .driveToAll(9.71, 2.52, 90)
    .driveToAll(15.7, 2.82, 130)
    .runFor(3, startShooting())
    .run(stopAll())
      .build()
      .withName(mirror ? "Blue Right" : "Red Left");
}



  private Command RightAuto(boolean mirror) {
    var routine = mirror
        ? AutoRoutine.createMirrored(swerve, localization)
        : AutoRoutine.create(swerve, localization);
    return routine
        .startAt(12.16, 7.383, 90)
        .run(startIntaking()) // don't cut off intaking early if we get stuck on the first waypoint
        .driveToAll(8.88, 7.321, 115)
        .driveToAll(8.88, 4.471, 115)
        .driveToAll(9.71, 5.67, 0,2.6)
        .driveToAll(14.7, 5.67, 180,2.6)
        .driveToAll(15.7, 5.2, 230)
        .runFor(3, startShooting())
        .run(stopShooting())
        .driveToAll(10.71, 5.75, 0)
        .driveToAll(9.51, 6.36, 90, 2.7)
        .driveToAll(9.38, 3.99, 90, 2.7)
        .driveToAll(10.58, 4, 230, 2.7)
        .driveToAll(9.71, 5.5, 270)
            .driveToAll(13.39, 5.67, 180)

        .driveToAll(15.7, 5.2, 230)
        .runFor(3, startShooting())
        .run(stopAll())
        .build()
        .withName(mirror ? "Blue Left" : "Red Right");
  }

  private Command MiddleAuto(boolean mirror) {
    var routine = mirror
        ? AutoRoutine.createMirrored(swerve, localization)
        : AutoRoutine.create(swerve, localization);
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
        ? AutoRoutine.createMirrored(swerve, localization)
        : AutoRoutine.create(swerve, localization);
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











  private Command RedRight() { return RightAuto(false); }
  private Command BlueRight() { return RightAuto(true); }


  private Command RedLeft() { return LeftAuto(false); }
  private Command BlueLeft() { return LeftAuto(true); }
  private Command RedMiddle() { return MiddleAuto(false); }
  private Command BlueMiddle() { return MiddleAuto(true); }
  private Command BlueMiddleDepot() {return MiddleAutoDepot(true);}
  private Command RedMiddleDepot() { return MiddleAutoDepot(false); }
   

}



package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.fms.FmsSubsystem;


public final class FieldPoints {
  private FieldPoints() {}

  public static final double FIELD_LENGTH = 16.5408;
  public static final double FIELD_WIDTH = 8.0692752;

  // ===== MIRRORING UTILITIES =====
  public static double mirrorX(double redX) {
    return FIELD_LENGTH - redX;
  }

  public static double mirrorDeg(double redDeg) {
    return 180.0 - redDeg;
  }

  public static Pose2d mirrorPose(Pose2d redPose) {
    return new Pose2d(
        FIELD_LENGTH - redPose.getX(),
        redPose.getY(),
        Rotation2d.fromDegrees(180.0 - redPose.getRotation().getDegrees()));
  }

  /** Mirrors a pose between the left and right sides while staying on the same alliance end. */
  public static Pose2d mirrorPoseLeftRight(Pose2d pose) {
    return new Pose2d(
        pose.getX(),
        FIELD_WIDTH - pose.getY(),
        Rotation2d.fromDegrees(-pose.getRotation().getDegrees()));
  }

  public static Translation2d mirrorTranslation(Translation2d redPoint) {
    return new Translation2d(FIELD_LENGTH - redPoint.getX(), redPoint.getY());
  }

  // ===== RED POSES (source of truth) =====

  // red neutral zone poses
  private static Pose2d RA1 = new Pose2d(12.955, 7.409, Rotation2d.kZero);
  private static Pose2d RB1 = new Pose2d(7.409,7.409, Rotation2d.kZero);
  private static Pose2d RA2 = new Pose2d(12.955, 0.674, Rotation2d.kZero);
  private static Pose2d RB2 = new Pose2d(7.409, 0.674, Rotation2d.kZero);

  // heading lock point (red is source of truth, blue is mirrored)
  private static Translation2d HEADINGLOCK_RED_POINT = new Translation2d(11.932568550109863, 4.2);

  // outpost pose (red is source of truth, blue is mirrored)
  private static Pose2d OUTPOST_RED = new Pose2d(16.25, 7.291, Rotation2d.fromDegrees(180.0));

  // Trench zones (X and Y bounds) — red values, use mirrorX() for blue
  public static final double TRENCH_X_MIN = 10.4;
  public static final double TRENCH_X_MAX = 13.2;
  public static final double RIGHT_TRENCH_Y_THRESHOLD = 6.8;
  public static final double RIGHT_TRENCH_Y_LOCK = 7.5;
  public static final double LEFT_TRENCH_Y_THRESHOLD = 1.3;
  public static final double LEFT_TRENCH_Y_LOCK = 0.6;

  // Shooter pose (robot-relative offset — same for both alliances)
  // Drum shooter aligns with robot heading and is centered on robot.
  public static final Pose2d SHOOTER_POSE = new Pose2d(0.0, 0.0, Rotation2d.kZero);

  // Pass target points (red values — blue derived via mirror)
  public static final Translation2d PASS_TARGET_RIGHT = new Translation2d(14.0, 7.0);
  public static final Translation2d PASS_TARGET_LEFT  = new Translation2d(14.0, 3.5);
  public static final Translation2d PASS_TARGET_RIGHT_BLUE = mirrorTranslation(PASS_TARGET_RIGHT);
  public static final Translation2d PASS_TARGET_LEFT_BLUE  = mirrorTranslation(PASS_TARGET_LEFT);

  // Shoot zone threshold (red X >= this means in shoot zone)
  public static final double SHOOT_X_THRESHOLD_RED = 11.0;

  // Y approach zones
  public static final double RIGHT_TRENCH_APPROACH_Y = 5.5;
  public static final double LEFT_TRENCH_APPROACH_Y = 2.5;

  // ===== ALLIANCE-AWARE GETTERS =====

  public static boolean isInShootZone(double robotX) {
    return isInShootZone(robotX, FmsSubsystem.isRedAlliance());
  }

  public static boolean isInShootZone(double robotX, boolean redAlliance) {
    if (redAlliance) {
      return robotX >= SHOOT_X_THRESHOLD_RED;
    } else {
      return robotX <= mirrorX(SHOOT_X_THRESHOLD_RED);
    }
  }

  public static Translation2d getAlliancePassTargetRight() {
    return FmsSubsystem.isRedAlliance() ? PASS_TARGET_RIGHT : PASS_TARGET_RIGHT_BLUE;
  }

  public static Translation2d getAlliancePassTargetLeft() {
    return FmsSubsystem.isRedAlliance() ? PASS_TARGET_LEFT : PASS_TARGET_LEFT_BLUE;
  }

  // ===== RED GETTERS =====
  public static Pose2d getRA1() { return RA1; }
  public static Pose2d getRB1() { return RB1; }
  public static Pose2d getRA2() { return RA2; }
  public static Pose2d getRB2() { return RB2; }

  // ===== BLUE GETTERS (mirrored from red on the fly) =====
  public static Pose2d getBA1() { return mirrorPose(RA1); }
  public static Pose2d getBB1() { return mirrorPose(RB1); }
  public static Pose2d getBA2() { return mirrorPose(RA2); }
  public static Pose2d getBB2() { return mirrorPose(RB2); }

  public static Translation2d getHeadingLockRedPoint() { return HEADINGLOCK_RED_POINT; }
  public static Translation2d getHeadingLockBluePoint() { return mirrorTranslation(HEADINGLOCK_RED_POINT); }

  public static Pose2d getOutpostRed() { return OUTPOST_RED; }
  public static Pose2d getOutpostBlue() { return mirrorPose(OUTPOST_RED); }

  // ===== RED SETTERS (blue auto-updates via mirror) =====
  public static void setRA1(Pose2d v) { RA1 = v; }
  public static void setRB1(Pose2d v) { RB1 = v; }
  public static void setRA2(Pose2d v) { RA2 = v; }
  public static void setRB2(Pose2d v) { RB2 = v; }

  public static void setHeadingLockRedPoint(Translation2d v) { HEADINGLOCK_RED_POINT = v; }

  public static void setOutpostRed(Pose2d v) { OUTPOST_RED = v; }

  public static void publishHeadingLockPoints() {
    SmartDashboard.putNumberArray("HeadingLock/RedPose",
        new double[]{HEADINGLOCK_RED_POINT.getX(), HEADINGLOCK_RED_POINT.getY(), 0.0});
    Translation2d bluePoint = getHeadingLockBluePoint();
    SmartDashboard.putNumberArray("HeadingLock/BluePose",
        new double[]{bluePoint.getX(), bluePoint.getY(), 0.0});
  }
}

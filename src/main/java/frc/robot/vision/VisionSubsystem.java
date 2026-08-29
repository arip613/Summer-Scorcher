package frc.robot.vision;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Timer;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.OptionalTagResult;
import java.util.function.DoubleSupplier;

public class VisionSubsystem extends StateMachine<VisionState> {
  /**
   * How much of the rewind buffer to flush at the end of each match period. The buffer caps at 165
   * seconds, so auto and teleop are captured separately rather than as one file -- a single capture
   * long enough to reach back past auto would not fit.
   */
  private static final double AUTO_REWIND_SECONDS = 30.0;

  private static final double TELEOP_REWIND_SECONDS = 160.0;
  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);

  private final Timer tableScanTimer = new Timer();

  private final ImuSubsystem imu;
  private final DoubleSupplier headingSupplier;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;
  private boolean wasAutoEnabled = false;
  private boolean wasTeleopEnabled = false;
  private boolean capturedAuto = false;
  private boolean capturedTeleop = false;

  private OptionalTagResult leftTagResult = new OptionalTagResult();
  private OptionalTagResult rightTagResult = new OptionalTagResult();

  private double robotHeading;
  private double angularVelocity;
  private double pitch;
  private double pitchRate;
  private double roll;
  private double rollRate;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;
  private boolean seenTagRecentlyForReset = true;

  public VisionSubsystem(ImuSubsystem imu, DoubleSupplier headingSupplier,
                         Limelight leftLimelight, Limelight rightLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.headingSupplier = headingSupplier;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
    tableScanTimer.start();
  }

  @Override
  protected void collectInputs() {
    angularVelocity = imu.getRobotAngularVelocity();
    robotHeading = headingSupplier.getAsDouble(); // CTRE offset-corrected heading for MT2
    pitch = imu.getPitch();
    pitchRate = imu.getPitchRate();
    roll = imu.getRoll();
    rollRate = imu.getRollRate();

    leftTagResult = leftLimelight.getTagResult();
    rightTagResult = rightLimelight.getTagResult();

    if (leftTagResult.isPresent() || rightTagResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }
    seeingTagDebounced = seeingTagDebouncer.calculate(seeingTag);
    if (RobotState.isDisabled()) {
      seenTagRecentlyForReset = true;
    } else {
      seenTagRecentlyForReset = seeingTagForPoseResetDebouncer.calculate(seeingTag);
    }
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getLeftTagResult() {
    return leftTagResult;
  }

  public OptionalTagResult getRightTagResult() {
    return rightTagResult;
  }

  public boolean seeingTagDebounced() {
    return seeingTagDebounced;
  }

  public boolean seenTagRecentlyForReset() {
    return seenTagRecentlyForReset;
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    leftLimelight.setState(LimelightState.TAGS);
    rightLimelight.setState(LimelightState.TAGS);
  }


  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    leftLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    rightLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);

    publishDiscoveredLimelightTables();
    captureRewindAtEndOfMatchPeriod();
  }

  /**
   * Flushes the cameras' rewind buffers to disk when each match period ends, on FMS only.
   *
   * <p>Rewind is always recording, so this costs nothing until it fires and nothing during the
   * match itself. Triggered on the enabled-to-disabled edge rather than on a timer: that is the
   * point where the period is definitively over and the robot has time to write the file.
   *
   * <p>FMS-gated on purpose. Firing on every practice enable would fill the cameras with .rwnd
   * files nobody asked for, and the point of this is having footage of real matches.
   */
  private void captureRewindAtEndOfMatchPeriod() {
    if (!RobotState.isFMSAttached()) {
      return;
    }

    if (RobotState.isAutonomousEnabled()) {
      wasAutoEnabled = true;
    } else if (wasAutoEnabled && !capturedAuto && RobotState.isDisabled()) {
      capturedAuto = true;
      triggerRewindOnBothCameras(AUTO_REWIND_SECONDS, "auto");
    }

    if (RobotState.isTeleopEnabled()) {
      wasTeleopEnabled = true;
    } else if (wasTeleopEnabled && !capturedTeleop && RobotState.isDisabled()) {
      capturedTeleop = true;
      triggerRewindOnBothCameras(TELEOP_REWIND_SECONDS, "teleop");
    }
  }

  private void triggerRewindOnBothCameras(double seconds, String label) {
    leftLimelight.triggerRewindCapture(seconds);
    rightLimelight.triggerRewindCapture(seconds);
    SmartDashboard.putString("Vision/LastRewindCapture", label + " " + seconds + "s");
    System.out.println("[Vision] Triggered " + seconds + "s rewind capture after " + label);
  }

  /**
   * Lists every root NetworkTables table whose name looks like a Limelight, so the names the
   * cameras actually publish under can be compared against the ones Hardware hardcodes.
   */
  private void publishDiscoveredLimelightTables() {
    if (!tableScanTimer.hasElapsed(1.0)) {
      return;
    }
    tableScanTimer.reset();

    var found = NetworkTableInstance.getDefault().getTable("/").getSubTables().stream()
        .filter(name -> name.toLowerCase().contains("limelight"))
        .sorted()
        .toArray(String[]::new);
    SmartDashboard.putStringArray("Vision/DiscoveredLimelightTables", found);
    SmartDashboard.putNumber("Vision/DiscoveredLimelightCount", found.length);
  }

  public void setClosestScoringReefAndPipe(int tagID) {
    leftLimelight.setClosestScoringReefTag(tagID);
    rightLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    return leftLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || rightLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyLeftScoringTagLimelightOnline() {
    return leftLimelight.isOnlineForTags();
  }

  public boolean isAnyRightScoringTagLimelightOnline() {
    return rightLimelight.isOnlineForTags();
  }

  public boolean isAnyTagLimelightOnline() {
    return leftLimelight.isOnlineForTags() || rightLimelight.isOnlineForTags();
  }
}

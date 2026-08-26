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
  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);

  private final Timer tableScanTimer = new Timer();

  private final ImuSubsystem imu;
  private final DoubleSupplier headingSupplier;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;

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

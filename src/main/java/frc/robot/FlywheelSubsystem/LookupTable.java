package frc.robot.FlywheelSubsystem;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.RobotState;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

public class LookupTable extends StateMachine<LookupTable.State> {

    public enum State { DISABLED, ENABLED }

    // Pass interp table: distance (m) → RPM
    // Match Hood's safe mechanical lower limit; older code requested -60 and was silently clamped.
    public static final double PASS_HOOD_ANGLE_DEG = -40.0;
    private static final InterpolatingDoubleTreeMap PASS_RPM_TABLE =
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(3.0, 3000.0  - 1000.0),
            Map.entry(5.0, 5000.0 - 1000.0),
            Map.entry(7.0, 5700.0),
            Map.entry(9.0, 6000.0));

    public static double getPassRpm(double distanceMeters) {
        return PASS_RPM_TABLE.get(distanceMeters);
    }

    public static class ShotPoint {
        public final double distanceMeters;
        public final double ta;
        public final double rpm;
        public final double hoodAngleDeg;

        public ShotPoint(double distanceMeters, double ta, double rpm, double hoodAngleDeg) {
            this.distanceMeters = distanceMeters;
            this.ta             = ta;
            this.rpm            = rpm;
            this.hoodAngleDeg   = hoodAngleDeg;
        }
    }

    @SuppressWarnings("unused")
    private static class TofPoint {
        final double distance;
        final double time;
        TofPoint(double distance, double time) { this.distance = distance; this.time = time; }
    }

    public record ShootingParameters(
            boolean    isValid,
            Rotation2d driveAngle,
            double     driveVelocityRadPerSec,
            double     hoodAngleRad,
            double     hoodVelocityRadPerSec,
            double     flywheelRpm,
            double     distance,
            double     distanceNoLookahead,
            double     timeOfFlight) {}

    // Manual override for building the shot table. With ManualOverride on, the lookup is bypassed
    // and the drum/hood follow the dashboard values, so a point can be dialled in live and then
    // recorded instead of edit-build-deploy per point.
    private static final String TUNE_MANUAL_KEY = "Shooter/Tune/ManualOverride";
    private static final String TUNE_RPM_KEY = "Shooter/Tune/RPM";
    private static final String TUNE_HOOD_KEY = "Shooter/Tune/HoodDeg";
    private static final double DEFAULT_TUNE_RPM = 2800.0;
    /**
     * Hood 0 is its rest position -- Hood's constructor calls setPosition(0.0) with no homing, so
     * zero is wherever the mechanism sits at boot. That rest position is the closest shot, and the
     * angle goes negative as distance increases. Starting here also means the first shot of a
     * tuning session falls short rather than sailing long.
     */
    private static final double DEFAULT_TUNE_HOOD_DEG = 0.0;

    private double lastActiveRpm = 0.0;
    private double lastActiveHoodDeg = 0.0;
    private final List<String> recordedPoints = new ArrayList<>();

    private static final double PHASE_DELAY_SECS = 0.03;
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 6.0;
    private static final Translation2d ROBOT_TO_LAUNCHER_TRANSLATION = new Translation2d(0.0, 0.0);
    private static final Rotation2d ROBOT_TO_LAUNCHER_ROTATION = Rotation2d.kZero;
    private static final int HOOD_FILTER_TAPS  = 20;
    private static final int DRIVE_FILTER_TAPS = 75;
    private static final double RPM_TOLERANCE      = 75.0;
    private static final double HOOD_TOLERANCE_DEG =  0.5;

    private final List<ShotPoint> shotPoints = new ArrayList<>();
    private final List<TofPoint>  tofPoints  = new ArrayList<>();

    private final LinearFilter hoodAngleFilter  = LinearFilter.movingAverage(HOOD_FILTER_TAPS);
    private final LinearFilter driveAngleFilter = LinearFilter.movingAverage(DRIVE_FILTER_TAPS);

    private double     lastHoodAngleRad = Double.NaN;
    private Rotation2d lastDriveAngle   = null;
    private double hoodAngleOffsetDeg = 0.0;
    private ShootingParameters cachedParameters = null;

    private boolean atGoal = false;
    private final Debouncer atGoalDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kFalling);

    private final DistanceCalc distanceCalc;
    private final Drum     drum;
    private final Hood         hood;

    public LookupTable(DistanceCalc distanceCalc, Drum drum, Hood hood) {
        super(SubsystemPriority.LOCALIZATION, State.DISABLED);
        this.distanceCalc = distanceCalc;
        this.drum     = drum;
        this.hood         = hood;
// do your job he
    //                      distance, tA,  RPM,  hood
     addShotPoint(new ShotPoint(1.32, 0.5, 2200, -15));
    addShotPoint(new ShotPoint(2.1,  0.39, 2250, -20));
     addShotPoint(new ShotPoint(2.5,  0.23, 2350, -23));
     addShotPoint(new ShotPoint(2.92936, 0.14, 2400, -33));
     addShotPoint(new ShotPoint(4,   0.08, 2750 + 80, -36));


        //addTofPoint(4, 0.4);
               // addTofPoint(5, 10000);

        // setDefault*, not put*: put* overwrites on every boot, which silently wiped the values
        // partway through a tuning session.
        SmartDashboard.setDefaultBoolean(TUNE_MANUAL_KEY, false);
        SmartDashboard.setDefaultNumber(TUNE_RPM_KEY, DEFAULT_TUNE_RPM);
        SmartDashboard.setDefaultNumber(TUNE_HOOD_KEY, DEFAULT_TUNE_HOOD_DEG);

        // All three persist so a redeploy mid-tuning-session does not lose them. Leaving the
        // override latched into a real match would silently ignore the shot table, so instead of
        // dropping it every boot it is force-cleared whenever an FMS is attached -- see
        // robotPeriodic. That keeps tuning frictionless without the competition footgun.
        var sd = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        sd.getEntry(TUNE_MANUAL_KEY).setPersistent();
        sd.getEntry(TUNE_RPM_KEY).setPersistent();
        sd.getEntry(TUNE_HOOD_KEY).setPersistent();

        SmartDashboard.putData("Shooter/Tune/RecordPoint", recordPointCommand());
        SmartDashboard.putString("Shooter/Tune/LastRecorded", "none yet");
    }

    /**
     * Captures the current pose distance with whatever RPM and hood are currently commanded, and
     * publishes it as a line that can be pasted straight into the constructor above.
     */
    private Command recordPointCommand() {
        return Commands.runOnce(
                () -> {
                    ShootingParameters p = getParameters();
                    String line =
                            String.format(
                                    "addShotPoint(new ShotPoint(%.2f, 0.0, %.0f, %.1f));",
                                    p.distanceNoLookahead(), lastActiveRpm, lastActiveHoodDeg);
                    recordedPoints.add(line);
                    SmartDashboard.putString("Shooter/Tune/LastRecorded", line);
                    SmartDashboard.putStringArray(
                            "Shooter/Tune/RecordedPoints", recordedPoints.toArray(new String[0]));
                    System.out.println("[ShotTune] " + line);
                })
            .ignoringDisable(true)
            .withName("ShotTune/RecordPoint");
    }

    public void addShotPoint(ShotPoint p) {
        shotPoints.add(p);
        shotPoints.sort(Comparator.comparingDouble(sp -> sp.distanceMeters));
    }

    public void clearCache() { cachedParameters = null; }

    public void enable()  { setStateFromRequest(State.ENABLED);  }
    public void disable() { setStateFromRequest(State.DISABLED); }

    public boolean isAtGoal() { return atGoal; }

    /**
     * The RPM and hood actually being commanded, which is the manual override when it is on and
     * the table otherwise. Callers must use these rather than re-deriving from getParameters(),
     * or they will fight this subsystem for control of the drum and hood.
     */
    public double getActiveRpm() { return lastActiveRpm; }

    public double getActiveHoodDeg() { return lastActiveHoodDeg; }

    public double getHoodAngleOffsetDeg() { return hoodAngleOffsetDeg; }
    public void incrementHoodAngleOffsetDeg(double deltaDeg) { hoodAngleOffsetDeg += deltaDeg; }

    public ShootingParameters getParameters() {
        if (cachedParameters != null) return cachedParameters;

        Pose2d estimatedPose = distanceCalc.getEstimatedPose();
        ChassisVelocities robotVel = distanceCalc.getRobotRelativeVelocity();
        estimatedPose = estimatedPose.plus(new Twist2d(
                robotVel.vx * PHASE_DELAY_SECS,
                robotVel.vy * PHASE_DELAY_SECS,
                robotVel.omega * PHASE_DELAY_SECS).exp());

        Translation2d launcherPos = estimatedPose.getTranslation()
                .plus(ROBOT_TO_LAUNCHER_TRANSLATION.rotateBy(estimatedPose.getRotation()));

        Translation2d target = distanceCalc.getAllianceTargetTranslation();
        double rawDistance = target.getDistance(launcherPos);

        ChassisVelocities launcherVel = computeLauncherVelocity(robotVel, estimatedPose.getRotation());

        Translation2d lookaheadLauncherPos = launcherPos;
        double lookaheadDistance = rawDistance;
        double timeOfFlight = lookupTof(rawDistance);

        for (int i = 0; i < 20; i++) {
            timeOfFlight = lookupTof(lookaheadDistance);
            double dx = launcherVel.vx * timeOfFlight;
            double dy = launcherVel.vy * timeOfFlight;
            lookaheadLauncherPos = launcherPos.plus(new Translation2d(dx, dy));
            lookaheadDistance    = target.getDistance(lookaheadLauncherPos);
        }

        Pose2d lookaheadRobotPose = new Pose2d(
                lookaheadLauncherPos.minus(
                        ROBOT_TO_LAUNCHER_TRANSLATION.rotateBy(estimatedPose.getRotation())),
                estimatedPose.getRotation());
        Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

        double[] shotParams   = lookupShot(lookaheadDistance);
        double   flywheelRpm  = shotParams[0];
        double   hoodAngleRad = Math.toRadians(shotParams[1]);

        if (lastDriveAngle == null)         lastDriveAngle   = driveAngle;
        if (Double.isNaN(lastHoodAngleRad)) lastHoodAngleRad = hoodAngleRad;

        double dt = 0.02;
        double hoodVelocity  = hoodAngleFilter.calculate((hoodAngleRad - lastHoodAngleRad) / dt);
        double driveVelocity = driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians() / dt);

        lastHoodAngleRad = hoodAngleRad;
        lastDriveAngle   = driveAngle;

        boolean valid = lookaheadDistance >= MIN_DISTANCE
                     && lookaheadDistance <= MAX_DISTANCE
                     && checkBadZones(estimatedPose);

        double hoodAngleWithOffset = hoodAngleRad + Math.toRadians(hoodAngleOffsetDeg);

        cachedParameters = new ShootingParameters(
                valid,
                driveAngle,
                driveVelocity,
                hoodAngleWithOffset,
                hoodVelocity,
                flywheelRpm,
                lookaheadDistance,
                rawDistance,
                timeOfFlight);

        SmartDashboard.putNumber("Shooter/TargetRPM",         flywheelRpm);
        SmartDashboard.putNumber("Shooter/TargetHoodDeg",     Math.toDegrees(hoodAngleWithOffset));
        SmartDashboard.putNumber("Shooter/TimeOfFlight",      timeOfFlight);
        SmartDashboard.putNumber("Shooter/DriveAngleDeg",     driveAngle.getDegrees());
        SmartDashboard.putBoolean("Shooter/IsValid",          valid);

        return cachedParameters;
    }

    @Override
    protected State getNextState(State current) { return current; }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        clearCache();

        // Published before the early return: everything under Shooter/Active* freezes at its last
        // value when nothing is being commanded, which reads as the override ignoring you.
        SmartDashboard.putBoolean("Shooter/Commanding", getState() == State.ENABLED);

        if (getState() != State.ENABLED) return;

        ShootingParameters p = getParameters();

        // RPM and hood come from the pose-based distance in getParameters(), which already applies
        // the velocity lookahead. The old tA path read limelight-left directly and overrode this
        // whenever a priority tag happened to be the camera's primary target, so the shot solution
        // silently switched sources mid-aim and ignored the lookahead.
        boolean manual = SmartDashboard.getBoolean(TUNE_MANUAL_KEY, false);
        if (manual && RobotState.isFMSAttached()) {
            // Never let a leftover tuning override run a real match.
            manual = false;
            SmartDashboard.putBoolean(TUNE_MANUAL_KEY, false);
        }
        double activeRpm;
        double activeHoodDeg;
        if (manual) {
            activeRpm = SmartDashboard.getNumber(TUNE_RPM_KEY, DEFAULT_TUNE_RPM);
            activeHoodDeg = SmartDashboard.getNumber(TUNE_HOOD_KEY, DEFAULT_TUNE_HOOD_DEG);
        } else {
            activeRpm = p.flywheelRpm();
            activeHoodDeg = Math.toDegrees(p.hoodAngleRad());
        }
        lastActiveRpm = activeRpm;
        lastActiveHoodDeg = activeHoodDeg;

        SmartDashboard.putBoolean("Shooter/ManualOverride", manual);
        SmartDashboard.putNumber("Shooter/LookupDistanceM", p.distance());
        SmartDashboard.putNumber("Shooter/RawDistanceM", p.distanceNoLookahead());
        // Shooter/Target* come from getParameters() and always show the table's answer. In manual
        // mode that is not what is being commanded, so publish the commanded values separately.
        SmartDashboard.putNumber("Shooter/ActiveRPM", activeRpm);
        SmartDashboard.putNumber("Shooter/ActiveHoodDeg", activeHoodDeg);

        // Always spin the drum so it's ready even if aim isn't valid yet
        drum.spinDrum(activeRpm);

        // Manual mode ignores the validity gate so a point can be tuned anywhere on the field.
        if (!manual && !p.isValid()) return;

        hood.setAngleDegrees(activeHoodDeg);

        boolean inTol =
        Math.abs(drum.getRpm()      - activeRpm)                  <= RPM_TOLERANCE
             && Math.abs(hood.getAngleDegrees() - activeHoodDeg)  <= HOOD_TOLERANCE_DEG;

        atGoal = atGoalDebouncer.calculate(inTol);
        SmartDashboard.putBoolean("Shooter/AtGoal", atGoal);
    }

    private double lookupTof(double distanceMeters) {
        if (tofPoints.isEmpty()) return 0.0;
        if (distanceMeters <= tofPoints.get(0).distance) return tofPoints.get(0).time;
        for (int i = 1; i < tofPoints.size(); i++) {
            TofPoint lo = tofPoints.get(i - 1), hi = tofPoints.get(i);
            if (distanceMeters <= hi.distance) {
                double t = (distanceMeters - lo.distance) / (hi.distance - lo.distance);
                return lo.time + t * (hi.time - lo.time);
            }
        }
        return tofPoints.get(tofPoints.size() - 1).time;
    }

    private double[] lookupShot(double distanceMeters) {
        if (shotPoints.isEmpty()) return new double[]{0, 0};
        if (distanceMeters <= shotPoints.get(0).distanceMeters) {
            ShotPoint p = shotPoints.get(0);
            return new double[]{p.rpm, p.hoodAngleDeg};
        }
        for (int i = 1; i < shotPoints.size(); i++) {
            ShotPoint lo = shotPoints.get(i - 1), hi = shotPoints.get(i);
            if (distanceMeters <= hi.distanceMeters) {
                double t = (distanceMeters - lo.distanceMeters) / (hi.distanceMeters - lo.distanceMeters);
                return new double[]{
                        lo.rpm          + t * (hi.rpm          - lo.rpm),
                        lo.hoodAngleDeg + t * (hi.hoodAngleDeg - lo.hoodAngleDeg)
                };
            }
        }
        ShotPoint last = shotPoints.get(shotPoints.size() - 1);
        return new double[]{last.rpm, last.hoodAngleDeg};
    }

    /** Get tA from the left limelight if a priority tag is visible. Returns null if no tag. */
    private ChassisVelocities computeLauncherVelocity(ChassisVelocities robotRelVel, Rotation2d robotAngle) {
        double fieldVx = robotRelVel.vx * robotAngle.getCos()
                       - robotRelVel.vy * robotAngle.getSin();
        double fieldVy = robotRelVel.vx * robotAngle.getSin()
                       + robotRelVel.vy * robotAngle.getCos();

        double omega = robotRelVel.omega;
        Translation2d offset = ROBOT_TO_LAUNCHER_TRANSLATION.rotateBy(robotAngle);
        fieldVx += -omega * offset.getY();
        fieldVy +=  omega * offset.getX();

        return new ChassisVelocities(fieldVx, fieldVy, omega);
    }

    private Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
        Rotation2d fieldToTargetAngle = target.minus(robotPose.getTranslation()).getAngle();
        return fieldToTargetAngle.plus(ROBOT_TO_LAUNCHER_ROTATION);
    }

    protected boolean checkBadZones(Pose2d estimatedPose) {
        return true;
    }

    public double getMinTimeOfFlight() { return lookupTof(MIN_DISTANCE); }

    public double getTimeOfFlightSeconds(double distanceMeters) {
        return lookupTof(distanceMeters);
    }

    public boolean hasCachedParameters() { return cachedParameters != null; }
    public boolean cachedParametersValid() { return cachedParameters != null && cachedParameters.isValid(); }
    public double getCachedFlywheelRpm() { return (cachedParameters != null) ? cachedParameters.flywheelRpm() : 0.0; }
    public double getCachedHoodAngleRad() { return (cachedParameters != null) ? cachedParameters.hoodAngleRad() : 0.0; }
    public double getCachedTimeOfFlight() { return (cachedParameters != null) ? cachedParameters.timeOfFlight() : 0.0; }
}

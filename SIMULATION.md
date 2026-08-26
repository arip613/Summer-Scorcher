# Robot simulation

Run the robot with:

```powershell
.\gradlew.bat simulateJava
```

In the Simulation Driver Station, select **Teleoperated** and enable the robot. The keyboard is
mapped as a 2027 WPILib Gamepad: WASD drives, left/right arrows rotate, E is the right trigger,
and Z/V/X/C control the POV. While disabled, selecting a blue Driver Station position resets to
`(2 m, 4 m, 0 deg)`; selecting a red position resets to its mirrored red-side pose.

Live logical controller axes are published under `/SmartDashboard/Simulation/Driver`. In
particular, moving the right stick horizontally must change `RightX`; if it does not, select the
physical controller in the Simulation GUI and enable its **Gamepad** mapping.

For controllers using the traditional raw Xbox layout, drivetrain rotation defaults to raw axis 4.
`RawAxes` shows every received axis, `RotationInput` shows the value actually sent to the
drivetrain, and `RotationAxis` can be changed at runtime (usually 4 for raw Xbox or 2 for a properly
mapped WPILib 2027 Gamepad).

`ShootAxis` selects the raw shoot-trigger axis (axis 3 by default), and `ShootInput` shows its live
value. The trigger uses the same field-position-based shoot/pass selection in simulation and on the
robot.

Heading lock initially aims from the simulated field pose. Inside its 10-degree acquisition window,
simulation supplies an ideal Limelight `tx` and publishes `HeadingLock/SimulatedTx=true`. With no
alliance selected, simulation defaults to blue to match the blue starting pose.

On the real robot, normal vision fusion uses CTRE state standard deviations of `(0.1 m, 0.1 m,
0.1 rad)`. After five seconds without an accepted pose, recovery is latched. Three fresh, consistent
strong observations at low angular velocity trigger one XY-only reset when disagreement is at least
0.25 m; heading remains gyro-based. Recovery state is published under `Localization/VisionRecovery`.

The drivetrain uses Phoenix's 5 ms swerve physics loop. The other TalonFX mechanisms use WPILib
DC motor models, and their combined current draw feeds the WPILib battery model. Useful values and
pose-reset commands are under `SmartDashboard/Simulation`; the field pose is under
`SmartDashboard/Field`.

AdvantageScope-compatible typed field topics are published at:

- `/Simulation/Field/RobotPose2d`
- `/Simulation/Field/RobotPose3d`
- `/Simulation/Field/RedHubPose`
- `/Simulation/Field/BlueHubPose`
- `/Simulation/Field/HubPoses` (`Pose3d[]`, red then blue)

In a 3D Field tab, use `RobotPose3d` for the robot and add `HubPoses` as a second object source.

To run autonomous, leave the robot disabled, choose a routine from `SmartDashboard/Auto Chooser`,
select **Autonomous** in the Simulation Driver Station, then enable. The selected routine resets its
own starting pose and follows its batched waypoints with BLine-Lib's WPILib 2027 Alpha 6 path
follower. `SmartDashboard/Auto/Selected` and `Auto/Running` show its status.

Phoenix 26.50.0-alpha-1's native desktop odometry thread is disabled in simulation because its
2027-alpha status waits block the robot loop. The acceleration-limited simulation pose supplies
localization and IMU feedback instead; the drivetrain integration test verifies rotation and
heading convergence.

To reproduce heading-lock behavior, enable teleop and hold the driver's right trigger while inside
the blue shooting zone. `HeadingLock/TargetAngleDeg`, `HeadingLock/HeadingErrorDeg`, and
`HeadingLock/Settled` show the controller's state.

Outside the alliance shooting zone, the same trigger enters pass mode. It chooses the nearer of the
two alliance pass targets, aims continuously from robot pose, selects flywheel RPM from pass
distance, commands the pass hood angle, and feeds only after heading, speed, hood, and flywheel are
ready. Inspect `Pass/TargetPose`, `Pass/RPM`, `Pass/HeadingError`, and `Pass/Ready`.

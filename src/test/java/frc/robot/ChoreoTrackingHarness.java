// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import java.io.File;
import java.nio.file.Files;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.Test;

/**
 * Headless harness that drives the real {@link Drive} + maple-sim physics through a Choreo
 * trajectory and records the X/Y tracking error each 20 ms tick to a CSV in {@code build/}. Not a
 * pass/fail unit test — it exists to generate tracking data that can be inspected directly.
 *
 * <p>Run with: {@code ./gradlew cleanTest test --tests frc.robot.ChoreoTrackingHarness}
 */
public class ChoreoTrackingHarness {
  private static final String TRAJECTORY = "Left_Trench";
  private static final double DT = 0.02;
  private static final double HOLD_SECONDS = 1.5; // time to observe end-of-path settling

  @Test
  public void measureTrackingError() throws Exception {
    HAL.initialize(500, 0);
    // NOTE: SimHooks timing control intentionally NOT used here — pausing the clock and advancing
    // it
    // in one 20 ms jump per loop appears to destabilize the Phoenix steer closed loop (which sub-
    // steps inside maple-sim). Letting the clock run continuously is the faithful behavior.
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    // Fresh drivetrain-only arena (bump collisions off, like Robot.java).
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SwerveDriveSimulation driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

    Drive drive =
        new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
            new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
            new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
            new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
    drive.setSimPoseConsumer(driveSimulation::setSimulationWorldPose);

    Trajectory<SwerveSample> traj = Choreo.<SwerveSample>loadTrajectory(TRAJECTORY).orElseThrow();
    SwerveSample start = traj.getInitialSample(false).orElseThrow();
    Pose2d finalPose = traj.getFinalPose(false).orElse(start.getPose());

    drive.setPose(start.getPose());
    drive.resetChoreoControllers();

    double total = traj.getTotalTime();
    StringBuilder csv = new StringBuilder();
    csv.append(
        "t,phase,sx,sy,sHeadDeg,truthX,truthY,truthHeadDeg,odomHeadDeg,errX,errY,errDist,headErrDeg\n");

    double maxErr = 0.0;
    double maxErrAfterEnd = 0.0;
    double finalErr = 0.0;
    double maxHeadErrDeg = 0.0;

    for (double t = 0.0; t <= total + HOLD_SECONDS + 1e-9; t += DT) {
      // Match the robot loop order: subsystem periodic (odometry) -> controller -> physics.
      Unmanaged.feedEnable(100);
      drive.periodic();

      boolean following = t <= total + 1e-9;
      double sampleTime = Math.min(t, total);
      SwerveSample s = traj.sampleAt(sampleTime, false).orElseThrow();
      if (following) {
        drive.followChoreoSample(s);
      } else {
        drive.holdPose(finalPose);
      }

      SimulatedArena.getInstance().simulationPeriodic();

      Pose2d truth = driveSimulation.getSimulatedDriveTrainPose();
      Pose2d odom = drive.getPose();
      double targetX = following ? s.x : finalPose.getX();
      double targetY = following ? s.y : finalPose.getY();
      double targetHead = following ? s.heading : finalPose.getRotation().getRadians();
      double errX = targetX - truth.getX();
      double errY = targetY - truth.getY();
      double errDist = Math.hypot(errX, errY);
      double headErrDeg =
          Math.toDegrees(MathUtil.angleModulus(targetHead - truth.getRotation().getRadians()));

      maxErr = Math.max(maxErr, errDist);
      if (!following) maxErrAfterEnd = Math.max(maxErrAfterEnd, errDist);
      finalErr = errDist;
      maxHeadErrDeg = Math.max(maxHeadErrDeg, Math.abs(headErrDeg));

      csv.append(
          String.format(
              "%.3f,%s,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f%n",
              t,
              following ? "follow" : "hold",
              targetX,
              targetY,
              Math.toDegrees(targetHead),
              truth.getX(),
              truth.getY(),
              Math.toDegrees(truth.getRotation().getRadians()),
              Math.toDegrees(odom.getRotation().getRadians()),
              errX,
              errY,
              errDist,
              headErrDeg));
    }

    File outDir = new File("build");
    outDir.mkdirs();
    File csvFile = new File(outDir, "choreo_tracking_" + TRAJECTORY + ".csv");
    Files.writeString(csvFile.toPath(), csv.toString());

    String summary =
        String.format(
            "Trajectory=%s  duration=%.3fs  maxErr=%.3fm  maxErrAfterPathEnd=%.3fm  finalErr=%.3fm  maxHeadErr=%.1fdeg%nCSV=%s%n",
            TRAJECTORY,
            total,
            maxErr,
            maxErrAfterEnd,
            finalErr,
            maxHeadErrDeg,
            csvFile.getAbsolutePath());
    Files.writeString(new File(outDir, "choreo_tracking_summary.txt").toPath(), summary);
    System.out.print(summary);
  }
}

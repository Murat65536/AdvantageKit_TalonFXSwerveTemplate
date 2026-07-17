// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Simulated motor controller for a TalonFX using MapleSim physics. */
  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  /** Simulated motor controller for a TalonFX with a remote CANcoder (steer motor). */
  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /** Returns simulated odometry timestamps for the current period. */
  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }
    return odometryTimeStamps;
  }

  /**
   * Regulates module constants for simulation. Adjusts inversions, encoder offsets, and PID gains
   * to work correctly with MapleSim physics. Has no effect on real robot.
   */
  public static SwerveModuleConstants regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    if (RobotBase.isReal()) return moduleConstants;

    return moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withDriveMotorGains(
            new Slot0Configs()
                .withKP(2.0)
                .withKI(0)
                .withKD(0.05)
                .withKS(moduleConstants.DriveFrictionVoltage)
                // kV must be in WHEEL (mechanism) units: ModuleIOTalonFX sets
                // Feedback.SensorToMechanismRatio = DriveMotorGearRatio, so the closed loop runs in
                // wheel rotations. Reuse the robot's characterized drive kV (~0.85 V/wheel-rps)
                // here
                // instead of the rotor-referenced 0.124, which was ~gearRatio too small and starved
                // the velocity feedforward at speed.
                .withKV(moduleConstants.DriveMotorGains.kV)
                // Acceleration feedforward in WHEEL units (V per wheel-rot/s^2). Model-derived for
                // the maple-sim Kraken X60 FOC: kA = R*(m/4)*r^2/(G*kt) * 2pi ~= 0.045, with
                // R = 12/483, kt = 9.37/483, m = robot mass, G = DriveMotorGearRatio, r = wheel
                // radius. Anticipates the trajectory's accel/decel so the robot tracks the final
                // braking and stops on the endpoint instead of coasting past it. Bump toward ~0.05
                // (rotor/wheel inertia) or refine with the wired SysId routines if overshoot
                // remains.
                .withKA(0.045)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign))
        .withSteerMotorGains(
            new Slot0Configs()
                // The template's kP=70 is unstable against the maple-sim steer model (low simulated
                // inertia + discrete loop): the azimuth oscillates past optimize()'s 180 deg flip
                // boundary and the modules spin, so the robot can't rotate at all. kP=15 is stable
                // and tracks pure rotation to ~3.9 rad/s. These gains were never validated on
                // hardware; re-tune on the real robot.
                .withKP(15)
                .withKI(0)
                .withKD(0.0)
                .withKS(moduleConstants.SteerFrictionVoltage)
                // Azimuth-referenced kV (FusedCANcoder mechanism). Reuse the characterized steer kV
                // (~1.16 for the X44) rather than the old 1.91 tuned for a slower steer motor.
                .withKV(moduleConstants.SteerMotorGains.kV)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
    // .withSteerMotorGearRatio(16.0)
    // .withDriveFrictionVoltage(Volts.of(0.1))
    // .withSteerFrictionVoltage(Volts.of(0.05))
    // .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}

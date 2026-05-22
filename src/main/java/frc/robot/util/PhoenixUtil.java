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

    // Drive velocity feedforward, in mechanism (wheel) rotation units. The drive closed loop
    // runs in mechanism units (SensorToMechanismRatio = drive gear ratio), so kV must be the
    // voltage that sustains the wheel's free speed: 12 V at SpeedAt12Volts. The prior hardcoded
    // 0.124 was a rotor-unit value (~gear-ratio times too small), which starved the velocity
    // loop and capped the simulated robot near 3 m/s instead of its rated SpeedAt12Volts.
    double driveKv =
        12.0 / (moduleConstants.SpeedAt12Volts / (2.0 * Math.PI * moduleConstants.WheelRadius));

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
                // kS (static friction) and kA (acceleration) are physical feedforward
                // constants of the drivetrain, the same in sim and real -- they match
                // TunerConstants.driveGains. The sim previously zeroed them, which dropped
                // friction compensation and acceleration anticipation; the missing kA
                // showed up as a heading-tracking lag during hard rotation (high alpha).
                .withKS(0.18)
                .withKV(driveKv)
                .withKA(0.06)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign))
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(70)
                .withKI(0)
                .withKD(4.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withSteerMotorGearRatio(16.0)
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.05))
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}

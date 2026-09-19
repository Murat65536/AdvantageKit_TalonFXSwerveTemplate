package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.rollers.RollerConfig;

/** Kicker: Neo Vortex on Spark Flex that injects fuel from the feeder into the flywheel. */
public final class KickerConstants {
  private KickerConstants() {}

  public static final int CAN_ID = 14;

  public static final RollerConfig CONFIG =
      new RollerConfig(
          "Kicker",
          CAN_ID,
          true,
          Amps.of(70),
          /* kP */ 0.0002,
          /* kI */ 0.0,
          /* kD */ 0.0,
          /* kV */ 0.0018,
          RPM.of(50));

  public static final AngularVelocity SHOOT_VELOCITY = RPM.of(3500);
  public static final AngularVelocity REVERSE_VELOCITY = RPM.of(-2000);
}

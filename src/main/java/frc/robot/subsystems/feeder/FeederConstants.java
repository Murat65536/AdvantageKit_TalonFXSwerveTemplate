package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.rollers.RollerConfig;

/** Feeder: Neo Vortex on Spark Flex that pushes fuel from the indexer into the kicker. */
public final class FeederConstants {
  private FeederConstants() {}

  public static final int CAN_ID = 6;

  public static final RollerConfig CONFIG =
      new RollerConfig(
          "Feeder", CAN_ID, false, Amps.of(70), 0.0001, 0.0, 0.0, 0.00185, RPM.of(100));

  public static final AngularVelocity FEED_VELOCITY = RPM.of(2000);
  public static final AngularVelocity REVERSE_VELOCITY = RPM.of(-2000);
}

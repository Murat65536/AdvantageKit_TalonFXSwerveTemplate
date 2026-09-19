package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.rollers.RollerConfig;

/** Indexer: Neo Vortex on Spark Flex that agitates the hopper and moves fuel to the feeder. */
public final class IndexerConstants {
  private IndexerConstants() {}

  public static final int CAN_ID = 7;

  public static final RollerConfig CONFIG =
      new RollerConfig(
          "Indexer",
          CAN_ID,
          true,
          Amps.of(70),
          /* kP */ 0.0002,
          /* kI */ 0.0,
          /* kD */ 0.0,
          /* kV */ 0.002,
          RPM.of(100));

  public static final AngularVelocity FORWARD_VELOCITY = RPM.of(1000);
  public static final AngularVelocity BACKWARD_VELOCITY = RPM.of(-1000);
}

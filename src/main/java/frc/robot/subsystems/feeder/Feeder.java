package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerSubsystem;

/** Moves fuel from the indexer up into the kicker. */
public class Feeder extends RollerSubsystem {
  public Feeder(RollerIO io) {
    super("Feeder", io, CONFIG.tolerance());
  }

  /** Feed fuel toward the kicker. Stops when the command ends. */
  public Command feed() {
    return runVelocity(FEED_VELOCITY).withName("FeederFeed");
  }

  /** Reverse to clear a jam. Stops when the command ends. */
  public Command reverse() {
    return runVelocity(REVERSE_VELOCITY).withName("FeederReverse");
  }
}

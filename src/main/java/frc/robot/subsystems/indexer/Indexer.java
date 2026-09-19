package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerSubsystem;

/** Agitates the hopper and moves fuel toward the feeder. */
public class Indexer extends RollerSubsystem {
  public Indexer(RollerIO io) {
    super("Indexer", io, CONFIG.tolerance());
  }

  /** Move fuel toward the feeder. Stops when the command ends. */
  public Command index() {
    return runVelocity(FORWARD_VELOCITY).withName("IndexerIndex");
  }

  /** Reverse to clear a jam or dispose fuel. Stops when the command ends. */
  public Command reverse() {
    return runVelocity(BACKWARD_VELOCITY).withName("IndexerReverse");
  }
}

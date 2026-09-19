package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerSubsystem;

/** Injects fuel from the feeder into the flywheel. Runs whenever the flywheel is spun up. */
public class Kicker extends RollerSubsystem {
  public Kicker(RollerIO io) {
    super("Kicker", io, CONFIG.tolerance());
  }

  /** Spin up to shooting speed. Stops when the command ends. */
  public Command shoot() {
    return runVelocity(SHOOT_VELOCITY).withName("KickerShoot");
  }

  /** Reverse to clear a jam. Stops when the command ends. */
  public Command reverse() {
    return runVelocity(REVERSE_VELOCITY).withName("KickerReverse");
  }
}

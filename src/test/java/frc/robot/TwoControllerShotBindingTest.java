package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManualShot;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The two-person shot (from the 2026RobotProject): the driver holds left trigger to aim and spin
 * up, the operator holds right trigger to feed. Both must run at once, so their commands may not
 * share a subsystem: the scheduler would cancel one when the other started, and the shot would
 * silently stop aiming or stop feeding.
 */
class TwoControllerShotBindingTest {
  private Drive drive;
  private Shooter shooter;
  private Hood hood;
  private Kicker kicker;
  private Feeder feeder;
  private Indexer indexer;
  private Intake intake;
  private Extension extension;

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    shooter = new Shooter(new ShooterIO() {});
    hood = new Hood(new HoodIO() {});
    kicker = new Kicker(new RollerIO() {});
    feeder = new Feeder(new RollerIO() {});
    indexer = new Indexer(new RollerIO() {});
    intake = new Intake(new IntakeIO() {});
    extension = new Extension(new ExtensionIO() {});
  }

  private static void assertDisjoint(Command a, Command b, String what) {
    for (Subsystem requirement : a.getRequirements()) {
      assertFalse(
          b.getRequirements().contains(requirement),
          what + " must not share requirement " + requirement.getName());
    }
  }

  @Test
  void driverAimAndSpinUpRunsAlongsideOperatorFeed() {
    Command driver =
        DriveCommands.joystickDriveAimAtTarget(drive, () -> 0.0, () -> 0.0)
            .alongWith(ShootCommands.spinUp(drive, shooter, hood, kicker));
    Command operator =
        ShootCommands.autoFeed(
            ShootCommands.readyToFeed(drive, shooter, hood), feeder, indexer, intake, extension);
    assertDisjoint(driver, operator, "driver left trigger and operator right trigger");
  }

  @Test
  void manualSpinRunsAlongsideManualHoodAndManualFeed() {
    ManualShot manual = new ManualShot(() -> true);
    Command spin = manual.spinShooter(shooter, kicker);
    Command holdHood = manual.holdHood(hood);
    Command feed = feeder.feed().alongWith(indexer.index());
    assertDisjoint(spin, holdHood, "manual spin (right trigger) and the manual hood");
    assertDisjoint(spin, feed, "manual spin (right trigger) and manual feed (B)");
    assertDisjoint(holdHood, feed, "the manual hood and manual feed (B)");
  }
}

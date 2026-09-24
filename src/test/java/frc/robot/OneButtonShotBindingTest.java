package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
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
import org.junit.jupiter.api.Test;

/**
 * Guards the single-controller one-button-shot binding: aiming the drive and running the shooting
 * pipeline must be able to run CONCURRENTLY. If ShootCommands.shootAtTarget ever starts requiring
 * the Drive subsystem, the command scheduler would cancel the aim command (or vice versa) and the
 * left-trigger binding would silently stop aiming while shooting.
 */
class OneButtonShotBindingTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void aimAndShootDoNotShareRequirements() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    Shooter shooter = new Shooter(new ShooterIO() {});
    Hood hood = new Hood(new HoodIO() {});
    Kicker kicker = new Kicker(new RollerIO() {});
    Feeder feeder = new Feeder(new RollerIO() {});
    Indexer indexer = new Indexer(new RollerIO() {});
    Intake intake = new Intake(new IntakeIO() {});
    Extension extension = new Extension(new ExtensionIO() {});

    Command aim = DriveCommands.joystickDriveAimAtTarget(drive, () -> 0.0, () -> 0.0);
    Command shoot =
        ShootCommands.shootAtTarget(
            drive, shooter, hood, kicker, feeder, indexer, intake, extension);

    assertTrue(aim.getRequirements().contains(drive), "aim command should require drive");
    assertFalse(
        shoot.getRequirements().contains(drive),
        "shootAtTarget must NOT require drive, or it will cancel the aim command");

    for (var requirement : shoot.getRequirements()) {
      assertFalse(
          aim.getRequirements().contains(requirement),
          "aim and shoot must not share requirement: " + requirement);
    }
  }
}

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.hood.HoodConstants.TRIM_STEP;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auton.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HubActivationWarningCommand;
import frc.robot.commands.ShootCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.choreo.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.extension.ExtensionIOReal;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOSim;
import frc.robot.subsystems.rollers.RollerIOSparkFlex;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int OPERATOR_CONTROLLER_PORT = 1;
  private static final double AIMING_RUMBLE = 0.8;

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Extension extension;
  private final Hood hood;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Feeder feeder;
  private final Indexer indexer;
  private final Autos autos;

  // Simulation
  private SwerveDriveSimulation driveSimulation = null;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operator =
      new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(LEFT_CAMERA_NAME, ROBOT_TO_LEFT_CAMERA_TRANSLATION),
                new VisionIOPhotonVision(RIGHT_CAMERA_NAME, ROBOT_TO_RIGHT_CAMERA_TRANSLATION));
        intake = new Intake(new IntakeIOReal());
        extension = new Extension(new ExtensionIOReal());
        hood = new Hood(new HoodIOReal());
        shooter = new Shooter(new ShooterIOReal());
        kicker = new Kicker(new RollerIOSparkFlex(KickerConstants.CONFIG));
        feeder = new Feeder(new RollerIOSparkFlex(FeederConstants.CONFIG));
        indexer = new Indexer(new RollerIOSparkFlex(IndexerConstants.CONFIG));
        break;

      case SIM:
        // Sim robot, instantiate MapleSim physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
        drive.setSimPoseConsumer(driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    LEFT_CAMERA_NAME,
                    ROBOT_TO_LEFT_CAMERA_TRANSLATION,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    RIGHT_CAMERA_NAME,
                    ROBOT_TO_RIGHT_CAMERA_TRANSLATION,
                    driveSimulation::getSimulatedDriveTrainPose));
        intake = new Intake(new IntakeIOSim(driveSimulation));
        extension = new Extension(new ExtensionIOSim());
        hood = new Hood(new HoodIOSim());
        kicker = new Kicker(new RollerIOSim(KickerConstants.CONFIG));
        feeder = new Feeder(new RollerIOSim(FeederConstants.CONFIG));
        indexer = new Indexer(new RollerIOSim(IndexerConstants.CONFIG));
        // In sim a fuel leaves the robot only when the feed path is actually pushing it into the
        // flywheel, mirroring the real robot.
        shooter =
            new Shooter(
                new ShooterIOSim(
                    driveSimulation::getSimulatedDriveTrainPose,
                    drive::getFieldRelativeChassisSpeeds,
                    hood::getAngle,
                    () ->
                        feeder.isRunningForward()
                            && kicker.isRunningForward()
                            && intake.consumeGamePiece()));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        extension = new Extension(new ExtensionIO() {});
        hood = new Hood(new HoodIO() {});
        shooter = new Shooter(new ShooterIO() {});
        kicker = new Kicker(new RollerIO() {});
        feeder = new Feeder(new RollerIO() {});
        indexer = new Indexer(new RollerIO() {});
        break;
    }

    autos = new Autos(drive, intake, extension, hood, shooter, kicker, feeder, indexer);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("No Auto", Commands.none());
    for (ChoreoTraj trajectory : autos.getAvailableTrajectories()) {
      autoChooser.addOption(
          "Choreo " + trajectory.name().replace('_', ' '), autos.buildTrajectoryAuto(trajectory));
    }

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureDriverBindings();
    configureOperatorBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverBindings() {
    // Default command: field-relative drive that slows down while driving into a bump
    drive.setDefaultCommand(
        DriveCommands.bumpAwareJoystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Re-seed field-centric heading: robot's front faces away from the driver station
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                    ? Rotation2d.kPi
                                    : Rotation2d.kZero)))
                .ignoringDisable(true));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // While B is held, extend and only run intake once extension is fully extended
    driver
        .b()
        .whileTrue(
            Commands.parallel(extension.extend(), intake.intake(extension::isFullyExtended)));

    // Obstacle align: snap heading for trench / bump while held
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveObstacleAlign(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));

    // Aim the shooter at the target while held; rumble both controllers while aiming
    driver
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAimAtTarget(
                    drive, () -> -driver.getLeftY(), () -> -driver.getLeftX())
                .alongWith(
                    Commands.startEnd(() -> setRumble(AIMING_RUMBLE), () -> setRumble(0.0))));

    // Full shot: spin up, aim hood, and auto-feed once ready
    driver.rightTrigger().whileTrue(shootAtTarget());
  }

  private void configureOperatorBindings() {
    // Intake / outtake (roller only)
    operator.rightBumper().whileTrue(intake.intake(extension::isFullyExtended));
    operator.leftBumper().whileTrue(intake.outtake());

    // Extension in / out
    operator.a().onTrue(extension.extend());
    operator.y().onTrue(extension.retract());

    // Full shot from the operator side as well
    operator.rightTrigger().whileTrue(shootAtTarget());

    // Clear a jam by reversing the whole fuel path
    operator.back().whileTrue(ShootCommands.unjam(shooter, kicker, feeder, indexer, intake));

    // Hood trim (applied on top of the calculated launch angle); Start resets it
    operator.povUp().onTrue(hood.adjustTrim(TRIM_STEP));
    operator.povDown().onTrue(hood.adjustTrim(TRIM_STEP.unaryMinus()));
    operator.start().onTrue(hood.resetTrim());
  }

  private Command shootAtTarget() {
    return ShootCommands.shootAtTarget(
        drive, shooter, hood, kicker, feeder, indexer, intake, extension);
  }

  private void setRumble(double value) {
    driver.getHID().setRumble(RumbleType.kBothRumble, value);
    operator.getHID().setRumble(RumbleType.kBothRumble, value);
  }

  /** Command that runs during teleop to warn drivers before the hub activates. */
  public Command getHubActivationWarningCommand() {
    return new HubActivationWarningCommand(this::setRumble);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Returns the current estimated robot pose. */
  public Pose2d getRobotPose() {
    return drive.getPose();
  }

  /** Returns number of game pieces currently stored in the intake simulation. */
  public int getStoredGamePieces() {
    return intake.getStoredGamePieces();
  }

  /** Returns poses of game pieces currently held by the intake simulation. */
  public Pose3d[] getHeldGamePiecePoses() {
    return intake.getHeldGamePiecePoses(drive.getPose());
  }

  /** Returns the current hood pose for simulation visualization. */
  public Pose3d getHoodPose() {
    return hood.getPose(drive.getPose());
  }

  public Pose2d getSimulatedDriveTrainPose() {
    return driveSimulation == null ? null : driveSimulation.getSimulatedDriveTrainPose();
  }
}

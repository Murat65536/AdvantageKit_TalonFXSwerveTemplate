// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auton.Autos;
import frc.robot.auton.StartPoseCheck;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriverRumble;
import frc.robot.commands.ManualShot;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.SystemsCheck;
import frc.robot.commands.TuningCommands;
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
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
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
  // Two controllers, as in the 2026RobotProject: the driver drives and aims, the operator runs
  // the fuel path and feeds the shot.
  private final CommandXboxController driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operator =
      new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  /** Both controllers rumble together, as in the old project. */
  private final DriverRumble rumble =
      new DriverRumble(
          level -> {
            driver.getHID().setRumble(RumbleType.kBothRumble, level);
            operator.getHID().setRumble(RumbleType.kBothRumble, level);
          });

  /** Manual mode (dashboard toggle): operator-set flywheel speed and hood angle. */
  private final ManualShot manualShot = new ManualShot();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** Blue-side start pose of each path-following auto, keyed by the chooser's command. */
  private final Map<Command, Pose2d> autoStartPosesBlue = new HashMap<>();

  private final Alert startPoseAlert = new Alert("", AlertType.kWarning);

  private final Dashboard dashboard;

  /**
   * Selected auto's start pose for our alliance, refreshed each loop by {@link #updateDashboard}.
   */
  private Optional<Pose2d> selectedAutoStart = Optional.empty();

  private boolean startPoseOk = true;

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
    addAuto(
        "Left Trench (2 volleys)", autos.leftTrench(), ChoreoTraj.Left_Trench.initialPoseBlue());
    addAuto(
        "Right Trench (2 volleys)", autos.rightTrench(), ChoreoTraj.Right_Trench.initialPoseBlue());
    addAuto(
        "Center Preload (8 fuel)",
        autos.centerPreload(),
        ChoreoTraj.Center_Preload.initialPoseBlue());
    addAuto(
        "Left Bump Depot (preload + depot)",
        autos.centerDepot(),
        ChoreoTraj.Center_Depot.initialPoseBlue());
    for (ChoreoTraj trajectory : autos.getAvailableTrajectories()) {
      addAuto(
          "Choreo " + trajectory.name().replace('_', ' '),
          autos.buildTrajectoryAuto(trajectory),
          trajectory.initialPoseBlue());
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
    autoChooser.addOption(
        "Tuning: Capture Tracking Noise (Mid-Range Shot)",
        TuningCommands.captureTrackingNoiseAtMidRangeShot(shooter, hood));
    autoChooser.addOption(
        "Tuning: Shot Dip @ 60deg 2500rpm (slip factor)",
        TuningCommands.captureShotDip(
            shooter, hood, kicker, feeder, indexer, intake, RPM.of(2500), Degrees.of(60.0)));
    autoChooser.addOption(
        "Tuning: Shot Dip @ 60deg 3000rpm (slip factor)",
        TuningCommands.captureShotDip(
            shooter, hood, kicker, feeder, indexer, intake, RPM.of(3000), Degrees.of(60.0)));
    autoChooser.addOption(
        "Tuning: Shot Dip @ 70deg 2500rpm (slip factor)",
        TuningCommands.captureShotDip(
            shooter, hood, kicker, feeder, indexer, intake, RPM.of(2500), Degrees.of(70.0)));

    dashboard =
        new Dashboard(
            SystemsCheck.standard(
                new SystemsCheck.PreferencesStore(),
                drive,
                vision,
                intake,
                indexer,
                feeder,
                kicker,
                shooter,
                hood,
                extension),
            hood);

    // Configure the button bindings
    configureBindings();
  }

  /**
   * Single-controller binding layout. Everything the robot does in a match is on the driver
   * controller; there is no operator controller.
   *
   * <p>Left trigger is the one-button shot: it aims the drive at the target AND runs the full
   * shooting pipeline at the same time. {@link ShootCommands#shootAtTarget} already gates feeding
   * on the drive actually being aimed (plus flywheel and hood at setpoint), so fuel only leaves the
   * robot once the alignment it is performing has converged.
   */
  private void configureBindings() {
    // Warn both drivers before our hub activates, for all of teleop.
    RobotModeTriggers.teleop().whileTrue(rumble.hubWarning());

    Trigger manualMode = new Trigger(manualShot::isEnabled);
    Trigger automaticMode = manualMode.negate();

    // ---------------- Driver (port 0) ----------------

    // Default command: field-relative drive that slows down while driving into a bump
    drive.setDefaultCommand(
        DriveCommands.bumpAwareJoystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Start: re-seed field-centric heading (robot's front faces away from the driver station)
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                FieldConstants.getAlliance() == Alliance.Red
                                    ? Rotation2d.kPi
                                    : Rotation2d.kZero)))
                .ignoringDisable(true));

    // X: lock the wheels in an X while held
    driver.x().whileTrue(drive.run(drive::stopWithX).withName("DriveXLock"));

    // Left bumper: snap heading for the trench / bump while held
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveObstacleAlign(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));

    // Left trigger: aim at the hub and spin up the shot (flywheel, kicker, hood) while held. The
    // operator feeds with right trigger. Off in manual mode, where the operator sets the shot.
    driver
        .leftTrigger()
        .and(automaticMode)
        .whileTrue(
            DriveCommands.joystickDriveAimAtTarget(
                    drive, () -> -driver.getLeftY(), () -> -driver.getLeftX())
                .alongWith(
                    ShootCommands.spinUp(drive, shooter, hood, kicker), rumble.whileAiming()));

    // ---------------- Operator (port 1) ----------------

    // Bumpers: outtake (left) and intake (right) while held. Intake runs once the hopper is out.
    operator.leftBumper().whileTrue(intake.outtake());
    operator.rightBumper().whileTrue(intake.intake(extension::isFullyExtended));

    // A / Y: hopper extension out / in
    operator.a().onTrue(extension.extend());
    operator.y().onTrue(extension.retract());

    // Right trigger: feed while held. Automatic mode feeds only once the shot is ready (the driver
    // must be holding left trigger to aim and spin up). Manual mode spins the flywheel at the
    // manual speed instead; feed with B.
    operator
        .rightTrigger()
        .and(automaticMode)
        .whileTrue(
            ShootCommands.autoFeed(
                ShootCommands.readyToFeed(drive, shooter, hood),
                feeder,
                indexer,
                intake,
                extension));
    operator.rightTrigger().and(manualMode).whileTrue(manualShot.spinShooter(shooter, kicker));

    // Manual mode only: X runs the kicker, B runs the feeder and indexer, while held
    operator.x().and(manualMode).whileTrue(kicker.shoot());
    operator.b().and(manualMode).whileTrue(feeder.feed().alongWith(indexer.index()));

    // D-pad: manual flywheel speed (up / down) and hood angle (right steeper / left shallower);
    // Start puts both back to their defaults
    operator.povUp().onTrue(manualShot.adjustRpm(ManualShot.RPM_STEP));
    operator.povDown().onTrue(manualShot.adjustRpm(-ManualShot.RPM_STEP));
    operator.povRight().onTrue(manualShot.adjustHood(ManualShot.HOOD_STEP_DEG));
    operator.povLeft().onTrue(manualShot.adjustHood(-ManualShot.HOOD_STEP_DEG));
    operator.start().onTrue(manualShot.reset());

    // In manual mode the hood holds the manual angle for all of teleop
    RobotModeTriggers.teleop().and(manualMode).whileTrue(manualShot.holdHood(hood));
  }

  private void addAuto(String name, Command command, Pose2d startPoseBlue) {
    autoChooser.addOption(name, command);
    autoStartPosesBlue.put(command, startPoseBlue);
  }

  /**
   * Call while disabled: warns on the dashboard when the robot is not where the selected auto
   * starts. Uses the estimated pose, so it is only as good as vision at the time of the check.
   */
  public void checkAutoStartPose() {
    Optional<Pose2d> start = selectedAutoStart();
    StartPoseCheck.Result result = StartPoseCheck.check(start, drive.getPose());
    startPoseOk = result.ok();
    Logger.recordOutput("Auto/StartPoseOk", result.ok());
    Logger.recordOutput("Auto/StartPoseErrorM", result.translationErrorMeters());
    Logger.recordOutput("Auto/StartHeadingErrorDeg", result.headingErrorDeg());
    start.ifPresent(pose -> Logger.recordOutput("Auto/StartPose", pose));
    startPoseAlert.setText(
        String.format(
            "Robot is %.0f cm / %.1f deg from the selected auto's start",
            result.translationErrorMeters() * 100, result.headingErrorDeg()));
    startPoseAlert.set(!result.ok());
  }

  private Optional<Pose2d> selectedAutoStart() {
    return Optional.ofNullable(autoStartPosesBlue.get(autoChooser.get()))
        .map(
            pose ->
                FieldConstants.getAlliance() == Alliance.Red
                    ? ChoreoAllianceFlipUtil.flip(pose)
                    : pose);
  }

  /** Call every loop: refreshes the Match and Pit tabs. */
  public void updateDashboard() {
    if (DriverStation.isEnabled()) {
      startPoseOk = true; // only meaningful while lining up, before the match
    }
    dashboard.update(drive.getPose(), selectedAutoStart(), startPoseOk, manualShot);
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

  /** Preloads game pieces into the intake in simulation. */
  public void preloadFuel(int count) {
    intake.addGamePieces(count);
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

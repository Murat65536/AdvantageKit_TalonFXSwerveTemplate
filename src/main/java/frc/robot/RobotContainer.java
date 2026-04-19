// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.HUB_TRANSLATION;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
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
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
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
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Extension extension;
  private final Hood hood;
  private final Shooter shooter;

  // Simulation
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
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
        break;

      case SIM:
        // Sim robot, instantiate MapleSim physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
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
        shooter =
            new Shooter(
                new ShooterIOSim(
                    driveSimulation::getSimulatedDriveTrainPose,
                    drive::getFieldRelativeChassisSpeeds,
                    hood::getAngle,
                    intake::consumeGamePiece));
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
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));
    // Joystick drive command

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // While B is held, extend and only run intake once extension is fully extended.
    controller
        .b()
        .whileTrue(
            Commands.parallel(extension.extend(), intake.intake(extension::isFullyExtended)));

    controller.leftBumper().whileTrue(extension.retract());
    controller.rightBumper().whileTrue(extension.extend());

    controller
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDriveFacingPoint(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> HUB_TRANSLATION));

    // Dynamically set shooter speed/angle model for hub shots while right trigger is held
    controller
        .rightTrigger()
        .whileTrue(shooter.shootAtHub(drive::getPose, drive::getFieldRelativeChassisSpeeds, hood));
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
}

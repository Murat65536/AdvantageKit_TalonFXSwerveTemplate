package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Voltage;
import java.util.stream.IntStream;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {
  private Voltage rollerVout = Volts.zero();
  private final IntakeSimulation mapleSimIntake;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    mapleSimIntake =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveTrain, Inches.of(25), Inches.of(6), IntakeSide.FRONT, 40);
    mapleSimIntake.register();
  }

  /** Returns the MapleSim IntakeSimulation for cross-subsystem access. */
  public IntakeSimulation getIntakeSimulation() {
    return mapleSimIntake;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = RPM.of(ROLLER_RPM_PER_VOLT * rollerVout.in(Volts));
    inputs.rollerVoltageOut = rollerVout;
    inputs.rollerConnected = true;

    // MapleSim intake: active when roller is spinning
    if (rollerVout.gte(Volts.of(3))) {
      mapleSimIntake.startIntake();
    } else {
      mapleSimIntake.stopIntake();
    }
    Logger.recordOutput("Sim/HeldFuel", mapleSimIntake.getGamePiecesAmount());
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    rollerVout = voltage;
  }

  @Override
  public int getStoredGamePieces() {
    return mapleSimIntake.getGamePiecesAmount();
  }

  @Override
  public boolean consumeGamePiece() {
    return mapleSimIntake.obtainGamePieceFromIntake();
  }

  @Override
  public Pose3d[] getHeldGamePiecePoses(Pose2d robotPose) {
    int storedGamePieces = getStoredGamePieces();
    return IntStream.range(0, storedGamePieces)
        .mapToObj(
            index -> {
              int column = index % 3;
              int layer = index / 3;
              int row = layer / 3;
              double yOffset =
                  switch (column) {
                    case 0 -> FUEL_DIAMETER;
                    case 1 -> 0.0;
                    default -> -FUEL_DIAMETER;
                  };
              return new Pose3d(robotPose)
                  .transformBy(
                      new Transform3d(
                          new Translation3d(
                              row * FUEL_DIAMETER, yOffset, 0.22 + layer % 3 * FUEL_DIAMETER),
                          new Rotation3d()));
            })
        .toArray(Pose3d[]::new);
  }
}

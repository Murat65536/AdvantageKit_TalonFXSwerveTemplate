package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.units.measure.Voltage;
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
}

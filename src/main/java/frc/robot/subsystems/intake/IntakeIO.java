package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setRollerVoltage(Voltage voltage) {}

  default int getStoredGamePieces() {
    return 0;
  }

  default boolean consumeGamePiece() {
    return false;
  }

  default Pose3d[] getHeldGamePiecePoses(Pose2d robotPose) {
    return new Pose3d[] {};
  }

  @AutoLog
  class IntakeIOInputs {
    public AngularVelocity rollerVelocity = RPM.zero();
    public Voltage rollerVoltageOut = Volts.zero();
    public Current rollerCurrentOut = Amps.zero();
    public Temperature rollerTemp = Celsius.zero();
    public boolean rollerConnected = false;
  }
}

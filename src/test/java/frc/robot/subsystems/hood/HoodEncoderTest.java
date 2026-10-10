package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class HoodEncoderTest {
  private static final double EPS = 1e-9;

  private static double angleForRaw(double rawEncoderDeg) {
    return encoderPositionToAngle(unwrapEncoderPosition(rawEncoderDeg)).in(Degrees);
  }

  @Test
  void endsOfTravelReadAsTheHardStops() {
    assertEquals(76.0, angleForRaw(0.0), EPS);
    assertEquals(50.0, angleForRaw(26.0), EPS);
  }

  @Test
  void readingPastTheLowStopStaysBelowFifty() {
    // On the robot the hood sat at a raw 33 and the old wrapped error kept pushing it down.
    assertEquals(43.0, angleForRaw(33.0), EPS);
  }

  @Test
  void readingPastTheHighStopStaysAboveSeventySix() {
    // A raw reading just under the period is just past full retraction, not near 24 deg.
    assertEquals(78.0, angleForRaw(ENCODER_POSITION_CONVERSION_FACTOR - 2.0), EPS);
  }

  @Test
  void fullTravelMoveDrivesTowardTheTarget() {
    // At 50 deg, asked for 76: the encoder must go down (toward 0), so the error is negative.
    double position = unwrapEncoderPosition(26.0);
    double target = MAX_HOOD_ANGLE.minus(MAX_HOOD_ANGLE).in(Degrees);
    assertEquals(-26.0, target - position, EPS);
  }
}

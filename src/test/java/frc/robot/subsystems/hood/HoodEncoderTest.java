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
  void zeroMakesTheSteepStopReadMaxAndTheTravelFollows() {
    // Zeroed at a raw 32.39 (the robot's reading on 2026-10-17, before calibration).
    double zero = 32.39;
    assertEquals(76.0, rawEncoderToAngle(zero, zero).in(Degrees), EPS);
    // 26 deg of hood travel toward shallow wraps the raw reading past the period.
    double shallowRaw = (zero + 26.0) % ENCODER_POSITION_CONVERSION_FACTOR;
    assertEquals(50.0, rawEncoderToAngle(shallowRaw, zero).in(Degrees), EPS);
    assertEquals(63.0, rawEncoderToAngle(zero + 13.0, zero).in(Degrees), EPS);
  }

  @Test
  void zeroedReadingsPastEitherStopStayContinuous() {
    double zero = 50.0; // near the end of the period, so both sides wrap
    double period = ENCODER_POSITION_CONVERSION_FACTOR;
    assertEquals(80.0, rawEncoderToAngle(zero - 4.0, zero).in(Degrees), EPS);
    assertEquals(45.0, rawEncoderToAngle((zero + 31.0) % period, zero).in(Degrees), EPS);
  }

  @Test
  void unzeroedReadingMatchesTheOldConversion() {
    assertEquals(43.61, rawEncoderToAngle(32.39, 0.0).in(Degrees), 1e-6);
  }

  @Test
  void fullTravelMoveDrivesTowardTheTarget() {
    // At 50 deg, asked for 76: the encoder must go down (toward 0), so the error is negative.
    double position = unwrapEncoderPosition(26.0);
    double target = MAX_HOOD_ANGLE.minus(MAX_HOOD_ANGLE).in(Degrees);
    assertEquals(-26.0, target - position, EPS);
  }
}

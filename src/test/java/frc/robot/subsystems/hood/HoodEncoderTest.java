package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

  // A Spark MAX absolute encoder, modelled in both orders the firmware might apply the zero
  // offset relative to inversion. rawRot is the sensor's own reading, [0, 1).
  private static double wrap(double rot) {
    return ((rot % 1.0) + 1.0) % 1.0;
  }

  private static double invert(double rot, boolean inverted) {
    return inverted ? wrap(1.0 - rot) : rot;
  }

  private static double reportOffsetBeforeInversion(double rawRot, double offsetRot, boolean inv) {
    return invert(wrap(rawRot - offsetRot), inv) * ENCODER_POSITION_CONVERSION_FACTOR;
  }

  private static double reportOffsetAfterInversion(double rawRot, double offsetRot, boolean inv) {
    return wrap(invert(rawRot, inv) - offsetRot) * ENCODER_POSITION_CONVERSION_FACTOR;
  }

  @Test
  void firstCandidateZeroesASparkThatOffsetsBeforeInversion() {
    for (double raw : new double[] {0.0, 0.123, 0.623, 0.999}) {
      for (double oldOffset : new double[] {0.0, 0.41, 0.97}) {
        double reported = reportOffsetBeforeInversion(raw, oldOffset, true);
        double offset = sparkZeroOffsetCandidates(reported, oldOffset, true)[0];
        double after = reportOffsetBeforeInversion(raw, offset, true);
        assertTrue(readsAsSteepStop(after), "raw " + raw + ", old " + oldOffset + ": " + after);
        assertEquals(76.0, reportedEncoderToAngle(after).in(Degrees), 1e-6);
      }
    }
  }

  @Test
  void secondCandidateZeroesASparkThatOffsetsAfterInversion() {
    for (double raw : new double[] {0.0, 0.123, 0.623, 0.999}) {
      for (double oldOffset : new double[] {0.0, 0.41, 0.97}) {
        double reported = reportOffsetAfterInversion(raw, oldOffset, true);
        double offset = sparkZeroOffsetCandidates(reported, oldOffset, true)[1];
        double after = reportOffsetAfterInversion(raw, offset, true);
        assertTrue(readsAsSteepStop(after), "raw " + raw + ", old " + oldOffset + ": " + after);
      }
    }
  }

  @Test
  void theWrongOrderIsCaughtByTheCheck() {
    // If the Spark offsets before inversion, candidate 1 must NOT read as zeroed (or the sequence
    // would keep a bad zero). raw 0.3 with no old offset: the two candidates differ.
    double reported = reportOffsetBeforeInversion(0.3, 0.0, true);
    double[] candidates = sparkZeroOffsetCandidates(reported, 0.0, true);
    assertFalse(readsAsSteepStop(reportOffsetBeforeInversion(0.3, candidates[1], true)));
  }

  @Test
  void bothCandidatesAgreeWhenNotInverted() {
    double[] c = sparkZeroOffsetCandidates(20.0, 0.25, false);
    assertEquals(c[0], c[1], EPS);
  }

  @Test
  void offsetsStayInTheSparksRange() {
    for (double reported : new double[] {0.0, 1.0, 25.9, 51.99}) {
      for (double c : sparkZeroOffsetCandidates(reported, 0.99, true)) {
        assertTrue(c >= 0.0 && c < 1.0, "offset " + c);
      }
    }
  }

  @Test
  void steepStopCheckAllowsTheWrapAndRejectsMidTravel() {
    assertTrue(readsAsSteepStop(0.4));
    assertTrue(readsAsSteepStop(ENCODER_POSITION_CONVERSION_FACTOR - 0.4));
    assertFalse(readsAsSteepStop(13.0));
    assertFalse(readsAsSteepStop(ZERO_VERIFY_TOLERANCE_DEG + 0.5));
  }

  @Test
  void fullTravelMoveDrivesTowardTheTarget() {
    // At 50 deg, asked for 76: the encoder must go down (toward 0), so the error is negative.
    double position = unwrapEncoderPosition(26.0);
    double target = MAX_HOOD_ANGLE.minus(MAX_HOOD_ANGLE).in(Degrees);
    assertEquals(-26.0, target - position, EPS);
  }
}

package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Auto volleys stop early once the hopper is known to be empty. Only the sim can count fuel: the
 * real robot has no fuel sensor, so its IO reports 0 stored pieces at all times. Treating that 0 as
 * "empty" ended every real-robot volley on its first loop, so auto drove without shooting.
 */
class IntakeEmptyTest {
  @Test
  void robotWithoutAFuelSensorIsNeverKnownEmpty() {
    Intake real = new Intake(new IntakeIO() {});
    assertFalse(real.isKnownEmpty());
  }

  @Test
  void countingIoWithNoFuelIsKnownEmpty() {
    Intake counted =
        new Intake(
            new IntakeIO() {
              @Override
              public boolean countsGamePieces() {
                return true;
              }
            });
    assertTrue(counted.isKnownEmpty());
  }
}

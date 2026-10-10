package frc.robot.auton;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The robot has to be placed where the selected auto starts. The path first drives under the trench
 * (Game Manual TU22 section 5.6: 50.34 in opening) with 6.8 cm of planned clearance at its tightest
 * point, so a robot set down 10 cm off would clip it.
 */
class StartPoseCheckTest {
  private static final Pose2d START = new Pose2d(4.43574, 7.4368, Rotation2d.kZero);

  @Test
  void exactPlacementIsOk() {
    assertTrue(StartPoseCheck.check(Optional.of(START), START).ok());
  }

  @Test
  void placementJustInsideTranslationToleranceIsOk() {
    Pose2d pose = new Pose2d(START.getX() + 0.04, START.getY(), Rotation2d.kZero);
    assertTrue(StartPoseCheck.check(Optional.of(START), pose).ok());
  }

  @Test
  void tenCentimetresOffFails() {
    Pose2d pose = new Pose2d(START.getX(), START.getY() - 0.10, Rotation2d.kZero);
    var result = StartPoseCheck.check(Optional.of(START), pose);
    assertFalse(result.ok());
    assertEquals(0.10, result.translationErrorMeters(), 1e-9);
  }

  @Test
  void rotatedTenDegreesFails() {
    Pose2d pose = new Pose2d(START.getTranslation(), Rotation2d.fromDegrees(10));
    assertFalse(StartPoseCheck.check(Optional.of(START), pose).ok());
  }

  @Test
  void headingErrorWrapsAround() {
    Pose2d start = new Pose2d(1, 1, Rotation2d.fromDegrees(179));
    Pose2d pose = new Pose2d(1, 1, Rotation2d.fromDegrees(-179));
    assertEquals(2.0, StartPoseCheck.check(Optional.of(start), pose).headingErrorDeg(), 1e-9);
  }

  @Test
  void autoWithoutAStartPoseNeverWarns() {
    assertTrue(StartPoseCheck.check(Optional.empty(), START).ok());
  }
}

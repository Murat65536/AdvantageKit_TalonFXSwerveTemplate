package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterMath;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Guards the contract between {@code tools/derive_shot_table.py} and {@link ShooterMath}.
 *
 * <p>The table is machine-generated: you measure the flywheel slip factor on the real robot, re-run
 * the script, and it rewrites the {@code shotTable.put} block in place. That only works if the
 * script can still find and parse what it is supposed to rewrite. These tests fail loudly if the
 * table is hand-edited into a shape the script no longer recognises, rather than letting the script
 * silently skip rows the next time someone regenerates it.
 *
 * <p>This does NOT re-derive the physics -- {@code ShotTableScoringTest} and {@code
 * ShotTableRealPhysicsTest} already check that every row scores.
 */
public class ShotTableToolContractTest {

  /** Must stay in sync with the regex in derive_shot_table.py's parse_committed_table(). */
  private static final Pattern ROW =
      Pattern.compile(
          "shotTable\\.put\\(Units\\.inchesToMeters\\((\\d+)\\),\\s*"
              + "new ShotParameters\\(([\\d.]+),\\s*([\\d.]+),\\s*([\\d.]+)\\)\\);");

  private static final Path SHOOTER_MATH =
      Path.of("src/main/java/frc/robot/subsystems/shooter/ShooterMath.java");
  private static final Path DERIVE_SCRIPT = Path.of("tools/derive_shot_table.py");

  private record TableRow(int distanceInches, double rpm, double angleDeg, double tofSeconds) {}

  private static List<TableRow> parseTable() throws IOException {
    String source = Files.readString(SHOOTER_MATH, StandardCharsets.UTF_8);
    Matcher m = ROW.matcher(source);
    List<TableRow> rows = new ArrayList<>();
    while (m.find()) {
      rows.add(
          new TableRow(
              Integer.parseInt(m.group(1)),
              Double.parseDouble(m.group(2)),
              Double.parseDouble(m.group(3)),
              Double.parseDouble(m.group(4))));
    }
    return rows;
  }

  @BeforeEach
  void setUp() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  void tearDown() {
    HAL.shutdown();
  }

  @Test
  void derivationScriptIsPresentAndDocumentsTheSlipFactor() throws IOException {
    assertTrue(
        Files.exists(DERIVE_SCRIPT),
        "tools/derive_shot_table.py is missing -- the shot table can no longer be regenerated "
            + "after measuring the slip factor");

    String script = Files.readString(DERIVE_SCRIPT, StandardCharsets.UTF_8);
    assertTrue(
        script.contains("--slip-factor"),
        "derive_shot_table.py must expose --slip-factor: it is the one measured input the whole "
            + "table scales with");
  }

  @Test
  void scriptCanStillParseTheCommittedTable() throws IOException {
    List<TableRow> rows = parseTable();
    assertTrue(
        rows.size() >= 10,
        "derive_shot_table.py's parser found only "
            + rows.size()
            + " rows in ShooterMath.java. If the table was reformatted, update the regex in "
            + "parse_committed_table() to match, or --write/--check will silently misbehave.");
  }

  @Test
  void tableIsSortedAndEvenlySpaced() throws IOException {
    List<TableRow> rows = parseTable();
    for (int i = 1; i < rows.size(); i++) {
      assertTrue(
          rows.get(i).distanceInches() > rows.get(i - 1).distanceInches(),
          "shot table distances must ascend (the script writes them in order); "
              + rows.get(i).distanceInches()
              + " follows "
              + rows.get(i - 1).distanceInches());
    }
    int step = rows.get(1).distanceInches() - rows.get(0).distanceInches();
    for (int i = 1; i < rows.size(); i++) {
      assertEquals(
          step,
          rows.get(i).distanceInches() - rows.get(i - 1).distanceInches(),
          "uneven spacing before " + rows.get(i).distanceInches() + "in");
    }
  }

  @Test
  void parsedRowsMatchWhatShooterMathServes() throws IOException {
    // Catches the table block and the min/max range drifting apart -- e.g. rows added by
    // hand without updating maxDistanceMeters, so clampDistance silently truncates them.
    for (TableRow row : parseTable()) {
      var served = ShooterMath.getShotParameters(Units.inchesToMeters(row.distanceInches()));
      assertEquals(
          row.rpm(),
          served.rpm(),
          1.0,
          String.format(
              "%din: table literal says %.0f rpm but ShooterMath serves %.0f -- the row is "
                  + "outside minDistanceMeters..maxDistanceMeters and is being clamped away",
              row.distanceInches(), row.rpm(), served.rpm()));
      assertEquals(
          row.angleDeg(),
          served.angleDeg(),
          0.05,
          String.format("%din: hood angle mismatch", row.distanceInches()));
      assertEquals(
          row.tofSeconds(),
          served.tofSeconds(),
          0.005,
          String.format("%din: TOF mismatch", row.distanceInches()));
    }
  }
}

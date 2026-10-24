package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ManualShot;
import frc.robot.commands.SystemsCheck;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Every widget in the deployed Elastic layout must point at a topic the robot publishes. A renamed
 * key would otherwise show as an empty widget in the pits, where nobody can fix it.
 */
class ElasticLayoutContractTest {
  private static final File LAYOUT = new File("src/main/deploy/elastic-layout.json");

  @Test
  void everyWidgetTopicIsPublished() throws Exception {
    HAL.initialize(500, 0);
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    // What the robot publishes: the Dashboard, the auto chooser, and the two alert groups.
    Dashboard dashboard =
        new Dashboard(new SystemsCheck(new SystemsCheck.MemoryStore()), new Hood(new HoodIO() {}));
    // The real toggle: ManualShot publishes Match/Manual Mode itself.
    dashboard.update(Pose2d.kZero, Optional.empty(), true, new ManualShot());
    new Extension(new ExtensionIO() {}); // publishes Match/Retract Delay
    new Shooter(new ShooterIO() {}); // publishes Match/Shot RPM Offset
    SmartDashboard.putData("Auto Choices", new SendableChooser<String>());
    new Alert("placeholder", AlertType.kInfo).set(true);
    new Alert(SystemsCheck.ALERT_GROUP, "placeholder", AlertType.kInfo).set(true);
    SmartDashboard.updateValues();
    nt.flushLocal();

    Set<String> published = new HashSet<>();
    for (Topic topic : nt.getTopics()) {
      published.add(topic.getName());
    }

    JsonNode layout = new ObjectMapper().readTree(LAYOUT);
    assertEquals(128, layout.get("grid_size").asInt());
    List<String> missing = new ArrayList<>();
    int widgets = 0;
    for (JsonNode tab : layout.get("tabs")) {
      for (JsonNode container : tab.get("grid_layout").get("containers")) {
        widgets++;
        String topic = container.get("properties").get("topic").asText();
        if (topic.equals("/FMSInfo")) {
          continue; // published by the DriverStation itself, not robot code
        }
        if (topic.startsWith("/CameraPublisher/")) {
          continue; // camera streams are published by PhotonVision on the coprocessor
        }
        // Sendables and alert groups publish sub-topics under their name.
        boolean found =
            published.contains(topic)
                || published.stream().anyMatch(name -> name.startsWith(topic + "/"));
        if (!found) {
          missing.add(
              tab.get("name").asText() + ": " + container.get("title").asText() + " -> " + topic);
        }
      }
    }
    assertTrue(widgets > 0, "layout has no widgets");
    assertTrue(
        missing.isEmpty(), "widgets with no published topic:\n" + String.join("\n", missing));
  }

  /**
   * Elastic reads x, y, width and height with {@code tryCast<double>}; a JSON integer (512) is not
   * a double in Dart, so it silently falls back to (0, 0) and a 1x1 cell, stacking every widget.
   * Each must be a JSON float on the 128 px grid, and no two widgets on a tab may overlap.
   */
  @Test
  void widgetsArePlacedOnTheGridWithoutOverlapping() throws Exception {
    JsonNode layout = new ObjectMapper().readTree(LAYOUT);
    List<String> problems = new ArrayList<>();
    for (JsonNode tab : layout.get("tabs")) {
      List<double[]> placed = new ArrayList<>();
      for (JsonNode c : tab.get("grid_layout").get("containers")) {
        String title = tab.get("name").asText() + "/" + c.get("title").asText();
        double[] rect = new double[4];
        String[] keys = {"x", "y", "width", "height"};
        for (int i = 0; i < 4; i++) {
          JsonNode v = c.get(keys[i]);
          if (v == null || !v.isFloatingPointNumber()) {
            problems.add(title + ": " + keys[i] + " must be a JSON float, was " + v);
            continue;
          }
          rect[i] = v.asDouble();
          if (rect[i] % 128.0 != 0.0 || (i >= 2 && rect[i] <= 0)) {
            problems.add(title + ": " + keys[i] + " = " + rect[i] + " is off the 128 px grid");
          }
        }
        for (double[] other : placed) {
          boolean overlap =
              rect[0] < other[0] + other[2]
                  && other[0] < rect[0] + rect[2]
                  && rect[1] < other[1] + other[3]
                  && other[1] < rect[1] + rect[3];
          if (overlap) {
            problems.add(title + " overlaps another widget");
          }
        }
        placed.add(rect);
      }
    }
    assertTrue(problems.isEmpty(), String.join("\n", problems));
  }
}

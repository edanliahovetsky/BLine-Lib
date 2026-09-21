package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class JsonUtilsTest {
    @TempDir
    java.nio.file.Path tempDir;

    @Test
    void publicLoaderReadsTankDirectionWithLegacyFallbackAndContextualErrors() throws IOException {
        writeConfig("{}");
        Files.createDirectories(tempDir.resolve("paths"));
        var file = tempDir.resolve("paths/Score Left.json");
        String elements = "\"path_elements\":[{\"type\":\"waypoint\",\"translation_target\":{\"x_meters\":1,\"y_meters\":2},\"rotation_target\":{\"rotation_radians\":0.7}}]";
        Files.writeString(file, "{" + elements + "}");
        assertEquals(DriveDirection.FORWARD, new Path(tempDir.toFile(), "Score Left").getTankDriveDirection());
        Files.writeString(file, "{\"tank_drive_direction\":\"backward\"," + elements + "}");
        Path backward = new Path(tempDir.toFile(), "Score Left");
        assertEquals(DriveDirection.BACKWARD, backward.getTankDriveDirection());
        assertEquals(.7, ((Path.Waypoint) backward.getPathElements().getFirst()).rotationTarget().rotation().getRadians(), 1e-9);
        Path transformed = backward.copy().setFlipped(true).setMirrored(true);
        assertEquals(DriveDirection.BACKWARD, transformed.getTankDriveDirection());
        transformed.setTankDriveDirection(DriveDirection.FORWARD);
        assertEquals(DriveDirection.BACKWARD, backward.getTankDriveDirection());
        Files.writeString(file, "{\"tank_drive_direction\":\"forward\"," + elements + "}");
        assertEquals(DriveDirection.FORWARD, new Path(tempDir.toFile(), "Score Left").getTankDriveDirection());
        assertEquals(DriveDirection.BACKWARD, backward.getTankDriveDirection(), "Loaded paths do not reread their files");
        for (String invalid : new String[] {"\"reverse\"", "\"FORWARD\"", "null", "false", "1", "{}"}) {
            Files.writeString(file, "{\"tank_drive_direction\":" + invalid + "," + elements + "}");
            var error = assertThrows(IllegalArgumentException.class, () -> new Path(tempDir.toFile(), "Score Left"));
            assertTrue(error.getMessage().contains("Score Left.json: tank_drive_direction"), error.getMessage());
        }
    }

    @Test
    void publicLoaderPreservesHandoffOverridesAndIdentifiesMalformedFields() throws IOException {
        writeConfig("{\"kinematic_constraints\":{\"default_handoff_mode\":\"progress\"}}");
        Files.createDirectories(tempDir.resolve("paths"));
        var file = tempDir.resolve("paths/Score Left.json");
        Files.writeString(file, """
            {"handoff_mode":"radius", "path_elements":[
              {"type":"translation", "x_meters":1, "y_meters":2,
               "handoff_mode":"progress", "intermediate_handoff_radius_meters":0.3}
            ]}
            """);
        var path = new Path(tempDir.toFile(), "Score Left.json");
        assertEquals(HandoffMode.PROGRESS, Path.getDefaultHandoffMode());
        assertEquals(HandoffMode.RADIUS, path.getHandoffMode().orElseThrow());
        var target = (Path.TranslationTarget) path.getPathElements().get(0);
        assertEquals(HandoffMode.PROGRESS, target.handoffMode().orElseThrow());
        assertEquals(.3, target.intermediateHandoffRadiusMeters().orElseThrow());
        assertTrue(new Path(tempDir.toFile(), "Score Left").isValid());

        Files.writeString(file, """
            {"path_elements":[{"type":"translation","y_meters":2}]}
            """);
        var error = assertThrows(IllegalArgumentException.class, () -> new Path(tempDir.toFile(), "Score Left"));
        assertTrue(error.getMessage().contains("Score Left.json"));
        assertTrue(error.getMessage().contains("element 1.translation.x_meters is missing"), error.getMessage());
        Files.writeString(file, """
            {"handoff_mode":"typo", "path_elements":[{"type":"translation","x_meters":1,"y_meters":2}]}
            """);
        assertTrue(assertThrows(IllegalArgumentException.class, () -> new Path(tempDir.toFile(), "Score Left"))
            .getMessage().contains("handoff_mode"));
        writeConfig("{}");
        Path.loadGlobalConstraints(tempDir.toFile());
        assertEquals(HandoffMode.RADIUS, Path.getDefaultHandoffMode(), "Old projects retain radius behavior");
    }

    @Test
    void loadGlobalConstraintsSupportsKinematicConstraintsSchema() throws IOException {
        String configJson = """
            {
              "gui": {
                "robot": {
                  "length_meters": 0.8,
                  "width_meters": 0.7
                }
              },
              "kinematic_constraints": {
                "default_max_velocity_meters_per_sec": 5.1,
                "default_max_acceleration_meters_per_sec2": 6.2,
                "default_max_velocity_deg_per_sec": 710.0,
                "default_max_acceleration_deg_per_sec2": 1410.0,
                "default_end_translation_tolerance_meters": 0.08,
                "default_end_rotation_tolerance_deg": 1.9,
                "default_intermediate_handoff_radius_meters": 0.33
              }
            }
            """;
        writeConfig(configJson);

        Path.DefaultGlobalConstraints constraints = JsonUtils.loadGlobalConstraints(tempDir.toFile());

        assertEquals(5.1, constraints.getMaxVelocityMetersPerSec(), 1e-9);
        assertEquals(6.2, constraints.getMaxAccelerationMetersPerSec2(), 1e-9);
        assertEquals(710.0, constraints.getMaxVelocityDegPerSec(), 1e-9);
        assertEquals(1410.0, constraints.getMaxAccelerationDegPerSec2(), 1e-9);
        assertEquals(0.08, constraints.getEndTranslationToleranceMeters(), 1e-9);
        assertEquals(1.9, constraints.getEndRotationToleranceDeg(), 1e-9);
        assertEquals(0.33, constraints.getIntermediateHandoffRadiusMeters(), 1e-9);
    }

    @Test
    void loadGlobalConstraintsFallsBackToRecursiveAliasLookup() throws IOException {
        String configJson = """
            {
              "future": {
                "layout": {
                  "profile": {
                    "max_velocity_meters_per_sec": 3.2,
                    "max_acceleration_meters_per_sec2": 4.4,
                    "max_velocity_deg_per_sec": 620.0,
                    "max_acceleration_deg_per_sec2": 1300.0,
                    "end_translation_tolerance_meters": 0.04,
                    "end_rotation_tolerance_deg": 2.3,
                    "intermediate_handoff_radius_meters": 0.27
                  }
                }
              }
            }
            """;
        writeConfig(configJson);

        Path.DefaultGlobalConstraints constraints = JsonUtils.loadGlobalConstraints(tempDir.toFile());

        assertEquals(3.2, constraints.getMaxVelocityMetersPerSec(), 1e-9);
        assertEquals(4.4, constraints.getMaxAccelerationMetersPerSec2(), 1e-9);
        assertEquals(620.0, constraints.getMaxVelocityDegPerSec(), 1e-9);
        assertEquals(1300.0, constraints.getMaxAccelerationDegPerSec2(), 1e-9);
        assertEquals(0.04, constraints.getEndTranslationToleranceMeters(), 1e-9);
        assertEquals(2.3, constraints.getEndRotationToleranceDeg(), 1e-9);
        assertEquals(0.27, constraints.getIntermediateHandoffRadiusMeters(), 1e-9);
    }

    @Test
    void loadGlobalConstraintsPrefersKinematicConstraintsOverLegacyTopLevelValues() throws IOException {
        String configJson = """
            {
              "default_max_velocity_meters_per_sec": 1.0,
              "default_max_acceleration_meters_per_sec2": 1.1,
              "default_max_velocity_deg_per_sec": 100.0,
              "default_max_acceleration_deg_per_sec2": 110.0,
              "default_end_translation_tolerance_meters": 0.2,
              "default_end_rotation_tolerance_deg": 9.0,
              "default_intermediate_handoff_radius_meters": 0.9,
              "kinematic_constraints": {
                "default_max_velocity_meters_per_sec": 5.6,
                "default_max_acceleration_meters_per_sec2": 7.7,
                "default_max_velocity_deg_per_sec": 780.0,
                "default_max_acceleration_deg_per_sec2": 1550.0,
                "default_end_translation_tolerance_meters": 0.03,
                "default_end_rotation_tolerance_deg": 1.2,
                "default_intermediate_handoff_radius_meters": 0.25
              }
            }
            """;
        writeConfig(configJson);

        Path.DefaultGlobalConstraints constraints = JsonUtils.loadGlobalConstraints(tempDir.toFile());

        assertEquals(5.6, constraints.getMaxVelocityMetersPerSec(), 1e-9);
        assertEquals(7.7, constraints.getMaxAccelerationMetersPerSec2(), 1e-9);
        assertEquals(780.0, constraints.getMaxVelocityDegPerSec(), 1e-9);
        assertEquals(1550.0, constraints.getMaxAccelerationDegPerSec2(), 1e-9);
        assertEquals(0.03, constraints.getEndTranslationToleranceMeters(), 1e-9);
        assertEquals(1.2, constraints.getEndRotationToleranceDeg(), 1e-9);
        assertEquals(0.25, constraints.getIntermediateHandoffRadiusMeters(), 1e-9);
    }

    @Test
    void loadPathFindsNestedConstraintKeysWithoutConstraintsBlock() throws ParseException {
        JSONObject pathJson = parseJson("""
            {
              "path_elements": [
                { "type": "translation", "x_meters": 0.0, "y_meters": 0.0 },
                { "type": "translation", "x_meters": 2.0, "y_meters": 0.0 }
              ],
              "future": {
                "constraints_bundle": {
                  "max_velocity_meters_per_sec": [
                    { "value": 2.5, "start_ordinal": 0, "end_ordinal": 1 }
                  ],
                  "max_acceleration_meters_per_sec2": [
                    { "value": 3.5, "start_ordinal": 0, "end_ordinal": 1 }
                  ],
                  "max_velocity_deg_per_sec": [
                    { "value": 400.0, "start_ordinal": 0, "end_ordinal": 1 }
                  ],
                  "max_acceleration_deg_per_sec2": [
                    { "value": 900.0, "start_ordinal": 0, "end_ordinal": 1 }
                  ],
                  "min_velocity_meters_per_sec": [
                    { "value": 0.7, "start_ordinal": 0, "end_ordinal": 1 }
                  ],
                  "min_velocity_deg_per_sec": [
                    { "value": 55.0, "start_ordinal": 0, "end_ordinal": 1 }
                  ],
                  "end_translation_tolerance_meters": 0.06,
                  "end_rotation_tolerance_deg": 2.7
                }
              }
            }
            """);

        Path path = JsonUtils.loadPath(
            pathJson,
            new Path.DefaultGlobalConstraints(4.0, 6.0, 720.0, 1440.0, 0.05, 2.0, 0.2)
        );
        Path.PathConstraints constraints = path.getPathConstraints();

        assertTrue(constraints.getMaxVelocityMetersPerSec().isPresent());
        assertEquals(2.5, constraints.getMaxVelocityMetersPerSec().get().get(0).value(), 1e-9);
        assertEquals(3.5, constraints.getMaxAccelerationMetersPerSec2().get().get(0).value(), 1e-9);
        assertEquals(400.0, constraints.getMaxVelocityDegPerSec().get().get(0).value(), 1e-9);
        assertEquals(900.0, constraints.getMaxAccelerationDegPerSec2().get().get(0).value(), 1e-9);
        assertEquals(0.7, constraints.getMinVelocityMetersPerSec().get().get(0).value(), 1e-9);
        assertEquals(55.0, constraints.getMinVelocityDegPerSec().get().get(0).value(), 1e-9);
        assertEquals(0.06, constraints.getEndTranslationToleranceMeters().orElseThrow(), 1e-9);
        assertEquals(2.7, constraints.getEndRotationToleranceDeg().orElseThrow(), 1e-9);
    }

    @Test
    void loadGlobalConstraintsFallsBackWhenSomeKeysMissing() throws IOException {
        String configJson = """
            {
              "kinematic_constraints": {
                "default_max_velocity_meters_per_sec": 6.0
              }
            }
            """;
        writeConfig(configJson);

        Path.DefaultGlobalConstraints constraints = JsonUtils.loadGlobalConstraints(tempDir.toFile());

        assertEquals(6.0, constraints.getMaxVelocityMetersPerSec(), 1e-9);
        assertEquals(7.0, constraints.getMaxAccelerationMetersPerSec2(), 1e-9);
        assertEquals(720.0, constraints.getMaxVelocityDegPerSec(), 1e-9);
        assertEquals(1500.0, constraints.getMaxAccelerationDegPerSec2(), 1e-9);
        assertEquals(0.03, constraints.getEndTranslationToleranceMeters(), 1e-9);
        assertEquals(2.0, constraints.getEndRotationToleranceDeg(), 1e-9);
        assertEquals(0.2, constraints.getIntermediateHandoffRadiusMeters(), 1e-9);
    }

    private void writeConfig(String json) throws IOException {
        Files.writeString(tempDir.resolve("config.json"), json, StandardCharsets.UTF_8);
    }

    private static JSONObject parseJson(String json) throws ParseException {
        return (JSONObject) new JSONParser().parse(json);
    }
}

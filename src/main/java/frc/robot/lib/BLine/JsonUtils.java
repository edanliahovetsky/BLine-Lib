package frc.robot.lib.BLine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

public class JsonUtils {
    /**
     * Container for parsed path components (without JSON parsing overhead in Path construction).
     * Use this to separate JSON parsing from Path construction for performance measurements.
     */
    public static record ParsedPathComponents(
        ArrayList<PathElement> elements,
        Path.PathConstraints constraints,
        Path.DefaultGlobalConstraints defaultGlobalConstraints
    ) {
        /**
         * Construct a Path from these parsed components.
         * This avoids JSON parsing overhead.
         */
        public Path toPath() {
            return new Path(elements, constraints, defaultGlobalConstraints);
        }
    }
    public static final File PROJECT_ROOT = new File(Filesystem.getDeployDirectory(), "autos");

    public static Path loadPath(File autosDir, String pathFileName) {
        try {
            File pathFile = new File(new File(autosDir, "paths"), pathFileName);

            // Read entire file to String (PathPlanner approach)
            String fileContent;
            try (BufferedReader br = new BufferedReader(new FileReader(pathFile))) {
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    sb.append(line);
                }
                fileContent = sb.toString();
            }

            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
            return buildPathFromJson(json, loadGlobalConstraints(autosDir));
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to load path from " + autosDir.getPath() + "/paths/" + pathFileName, e);
        }
    }

    public static Path loadPath(JSONObject json, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        return buildPathFromJson(json, defaultGlobalConstraints);
    }

    public static Path loadPath(String pathFileName) {
        return loadPath(PROJECT_ROOT, pathFileName);
    }

    public static Path loadPathFromJsonString(String pathJson, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        try {
            JSONObject json = (JSONObject) new JSONParser().parse(pathJson);
            return buildPathFromJson(json, defaultGlobalConstraints);
        } catch (ParseException e) {
            throw new RuntimeException("Failed to parse path JSON string", e);
        }
    }

    /**
     * Parse a path JSON object into components without constructing a Path.
     * This is useful for performance measurements where you want to separate
     * JSON parsing from Path construction.
     * 
     * @param pathJson The JSON object representing the path
     * @param defaultGlobalConstraints Optional default global constraints (can be null)
     * @return ParsedPathComponents containing elements, constraints, and globals
     */
    public static ParsedPathComponents parsePathComponents(JSONObject pathJson, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        ArrayList<PathElement> elements = parsePathElements(pathJson);
        Path.PathConstraints constraints = parsePathConstraints(pathJson);
        
        Path.DefaultGlobalConstraints globals = defaultGlobalConstraints;
        JSONObject globalsJson = (JSONObject) pathJson.get("default_global_constraints");
        if (globalsJson != null) {
            globals = parseDefaultGlobalConstraints(globalsJson);
        } else if (globals == null) {
            globals = loadGlobalConstraints(PROJECT_ROOT);
        }
        
        return new ParsedPathComponents(elements, constraints, globals);
    }

    private static Path buildPathFromJson(JSONObject json, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        ArrayList<PathElement> elements = parsePathElements(json);

        Path.PathConstraints constraints = parsePathConstraints(json);

        Path.DefaultGlobalConstraints globals = defaultGlobalConstraints;

        JSONObject globalsJson = (JSONObject) json.get("default_global_constraints");
        if (globalsJson != null) {
            globals = parseDefaultGlobalConstraints(globalsJson);
        } else if (globals == null) {
            globals = loadGlobalConstraints(PROJECT_ROOT);
        }

        return new Path(elements, constraints, globals);
    }

    private static ArrayList<PathElement> parsePathElements(JSONObject json) {
        ArrayList<PathElement> elements = new ArrayList<>();
        JSONArray pathElementsJson = (JSONArray) json.get("path_elements");
        if (pathElementsJson == null) {
            return elements;
        }

        for (Object obj : pathElementsJson) {
            if (!(obj instanceof JSONObject)) {
                continue;
            }
            JSONObject elementJson = (JSONObject) obj;
            String type = (String) elementJson.get("type");

            if ("translation".equals(type)) {
                double xMeters = ((Number) elementJson.get("x_meters")).doubleValue();
                double yMeters = ((Number) elementJson.get("y_meters")).doubleValue();
                Object handoffObj = elementJson.get("intermediate_handoff_radius_meters");
                Double handoff = handoffObj != null ? ((Number) handoffObj).doubleValue() : null;

                elements.add(new TranslationTarget(
                    new Translation2d(xMeters, yMeters),
                    Optional.ofNullable(handoff)
                ));
            } else if ("rotation".equals(type)) {
                double rotationRadians = ((Number) elementJson.get("rotation_radians")).doubleValue();
                Object tRatioObj = elementJson.get("t_ratio");
                double tRatio = tRatioObj != null ? ((Number) tRatioObj).doubleValue() : 0.5;
                Object profiledObj = elementJson.get("profiled_rotation");
                boolean profiled = profiledObj != null && (Boolean) profiledObj;

                elements.add(new RotationTarget(
                    Rotation2d.fromRadians(rotationRadians),
                    tRatio,
                    profiled
                ));
            } else if ("waypoint".equals(type)) {
                JSONObject translationJson = (JSONObject) elementJson.get("translation_target");
                if (translationJson == null) {
                    continue;
                }
                double txMeters = ((Number) translationJson.get("x_meters")).doubleValue();
                double tyMeters = ((Number) translationJson.get("y_meters")).doubleValue();
                Object tHandoffObj = translationJson.get("intermediate_handoff_radius_meters");
                Double tHandoff = tHandoffObj != null ? ((Number) tHandoffObj).doubleValue() : null;

                JSONObject rotationJson = (JSONObject) elementJson.get("rotation_target");
                if (rotationJson == null) {
                    continue;
                }
                double rotRadians = ((Number) rotationJson.get("rotation_radians")).doubleValue();
                Object rTRatioObj = rotationJson.get("t_ratio");
                double rTRatio = rTRatioObj != null ? ((Number) rTRatioObj).doubleValue() : 0.5;
                Object rProfiledObj = rotationJson.get("profiled_rotation");
                boolean rProfiled = rProfiledObj != null && (Boolean) rProfiledObj;

                TranslationTarget t = new TranslationTarget(
                    new Translation2d(txMeters, tyMeters),
                    Optional.ofNullable(tHandoff)
                );
                RotationTarget r = new RotationTarget(
                    Rotation2d.fromRadians(rotRadians),
                    rTRatio,
                    rProfiled
                );
                elements.add(new Waypoint(t, r));
            }
        }
        return elements;
    }

    private static Path.PathConstraints parsePathConstraints(JSONObject json) {
        Path.PathConstraints constraints = new Path.PathConstraints();
        JSONObject constraintsJson = (JSONObject) json.get("constraints");
        if (constraintsJson != null) {
            parseConstraint(constraintsJson, "max_velocity_meters_per_sec", (val) -> {
                if (val.isPresent()) constraints.setMaxVelocityMetersPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            });
            parseConstraint(constraintsJson, "max_acceleration_meters_per_sec2", (val) -> {
                if (val.isPresent()) constraints.setMaxAccelerationMetersPerSec2(val.get().toArray(new Path.RangedConstraint[0]));
            });
            parseConstraint(constraintsJson, "max_velocity_deg_per_sec", (val) -> {
                if (val.isPresent()) constraints.setMaxVelocityDegPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            });
            parseConstraint(constraintsJson, "max_acceleration_deg_per_sec2", (val) -> {
                if (val.isPresent()) constraints.setMaxAccelerationDegPerSec2(val.get().toArray(new Path.RangedConstraint[0]));
            });
            Object endTranslationTolObj = constraintsJson.get("end_translation_tolerance_meters");
            if (endTranslationTolObj instanceof Number) {
                constraints.setEndTranslationToleranceMeters(((Number) endTranslationTolObj).doubleValue());
            }
            Object endRotationTolObj = constraintsJson.get("end_rotation_tolerance_deg");
            if (endRotationTolObj instanceof Number) {
                constraints.setEndRotationToleranceDeg(((Number) endRotationTolObj).doubleValue());
            }
        }
        return constraints;
    }

    private static Path.DefaultGlobalConstraints parseDefaultGlobalConstraints(JSONObject json) {
        double dMaxVelMps = ((Number) json.get("default_max_velocity_meters_per_sec")).doubleValue();
        double dMaxAccMps2 = ((Number) json.get("default_max_acceleration_meters_per_sec2")).doubleValue();
        double dMaxVelDeg = ((Number) json.get("default_max_velocity_deg_per_sec")).doubleValue();
        double dMaxAccDeg2 = ((Number) json.get("default_max_acceleration_deg_per_sec2")).doubleValue();
        double endTransTol = ((Number) json.get("default_end_translation_tolerance_meters")).doubleValue();
        double endRotTolDeg = ((Number) json.get("default_end_rotation_tolerance_deg")).doubleValue();
        double handoffRadius = ((Number) json.get("default_intermediate_handoff_radius_meters")).doubleValue();

        return new Path.DefaultGlobalConstraints(
            dMaxVelMps,
            dMaxAccMps2,
            dMaxVelDeg,
            dMaxAccDeg2,
            endTransTol,
            endRotTolDeg,
            handoffRadius
        );
    }

    private static void parseConstraint(JSONObject constraintsJson, String key, java.util.function.Consumer<Optional<ArrayList<Path.RangedConstraint>>> setter) {
        JSONArray arr = (JSONArray) constraintsJson.get(key);
        if (arr != null && !arr.isEmpty()) {
            ArrayList<Path.RangedConstraint> list = new ArrayList<>();
            for (Object obj : arr) {
                JSONObject rcJson = (JSONObject) obj;
                double value = ((Number) rcJson.get("value")).doubleValue();
                int startOrdinal = ((Number) rcJson.get("start_ordinal")).intValue();
                int endOrdinal = ((Number) rcJson.get("end_ordinal")).intValue();
                list.add(new Path.RangedConstraint(value, startOrdinal, endOrdinal));
            }
            setter.accept(Optional.of(list));
        }
    }

    public static Path.DefaultGlobalConstraints loadGlobalConstraints(File autosDir) {
        try {
            File config = new File(autosDir, "config.json");

            // Read entire file to String (PathPlanner approach)
            String fileContent;
            try (BufferedReader br = new BufferedReader(new FileReader(config))) {
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    sb.append(line);
                }
                fileContent = sb.toString();
            }

            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            double dMaxVelMps = ((Number) json.get("default_max_velocity_meters_per_sec")).doubleValue();
            double dMaxAccMps2 = ((Number) json.get("default_max_acceleration_meters_per_sec2")).doubleValue();
            double dMaxVelDeg = ((Number) json.get("default_max_velocity_deg_per_sec")).doubleValue();
            double dMaxAccDeg2 = ((Number) json.get("default_max_acceleration_deg_per_sec2")).doubleValue();
            double endTransTol = ((Number) json.get("default_end_translation_tolerance_meters")).doubleValue();
            double endRotTolDeg = ((Number) json.get("default_end_rotation_tolerance_deg")).doubleValue();
            double handoffRadius = ((Number) json.get("default_intermediate_handoff_radius_meters")).doubleValue();

            return new Path.DefaultGlobalConstraints(
                dMaxVelMps,
                dMaxAccMps2,
                dMaxVelDeg,
                dMaxAccDeg2,
                endTransTol,
                endRotTolDeg,
                handoffRadius
            );
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to load global constraints from " + autosDir.getPath() + "/config.json", e);
        }
    }

}

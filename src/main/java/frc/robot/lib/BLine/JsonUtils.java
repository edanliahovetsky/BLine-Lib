package frc.robot.lib.BLine;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Filesystem;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.EventTrigger;
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
import java.util.function.Consumer;

/** Internal codec for the public {@link Path} loading API. */
final class JsonUtils {
    private JsonUtils() {}

    private static final String[][] PATH_CONSTRAINT_KEY_ALIASES = {
        { "max_velocity_meters_per_sec" },
        { "max_acceleration_meters_per_sec2" },
        { "max_velocity_deg_per_sec" },
        { "max_acceleration_deg_per_sec2" },
        { "min_velocity_meters_per_sec" },
        { "min_velocity_deg_per_sec" },
        { "end_translation_tolerance_meters" },
        { "end_rotation_tolerance_deg" }
    };

    private static final String[][] GLOBAL_CONSTRAINT_KEY_ALIASES = {
        { "default_max_velocity_meters_per_sec", "max_velocity_meters_per_sec" },
        { "default_max_acceleration_meters_per_sec2", "max_acceleration_meters_per_sec2" },
        { "default_max_velocity_deg_per_sec", "max_velocity_deg_per_sec" },
        { "default_max_acceleration_deg_per_sec2", "max_acceleration_deg_per_sec2" },
        { "default_end_translation_tolerance_meters", "end_translation_tolerance_meters" },
        { "default_end_rotation_tolerance_deg", "end_rotation_tolerance_deg" },
        { "default_intermediate_handoff_radius_meters", "intermediate_handoff_radius_meters" }
    };

    private static final Path.DefaultGlobalConstraints FALLBACK_GLOBAL_CONSTRAINTS =
        new Path.DefaultGlobalConstraints(4.5, 7.0, 720.0, 1500.0, 0.03, 2.0, 0.2);

    static File projectRoot() {
        try {
            return new File(Filesystem.getDeployDirectory(), "autos");
        } catch (Throwable ignored) {
            // Allows unit tests or desktop environments that don't include camera server classes.
            return new File("src/main/deploy/autos");
        }
    }

    /**
     * Loads a path from a JSON file in the specified autos directory.
     * 
     * <p>The path file should be located in a "paths" subdirectory within the autos directory.
     * Global constraints are loaded from a "config.json" file in the autos directory.
     * 
     * @param autosDir The directory containing the autos (with paths/ subdirectory)
     * @param pathFileName The name of the path file (including .json extension)
     * @return The loaded Path object
     * @throws RuntimeException if the file cannot be read or parsed
     */
    static Path loadPath(File autosDir, String pathFileName) {
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

            JSONObject json = object(new JSONParser().parse(fileContent), "path");
            ProjectConfig config = readProjectConfig(autosDir);
            Path path = buildPathFromJson(json, config.defaults());
            Path.setDefaultHandoffMode(config.handoffMode());
            return path;
        } catch (IOException | ParseException | RuntimeException e) {
            throw new IllegalArgumentException(pathFileName + ": " + e.getMessage(), e);
        }
    }

    /**
     * Loads a path from a pre-parsed JSON object with specified global constraints.
     * 
     * @param json The parsed JSON object representing the path
     * @param defaultGlobalConstraints The default global constraints to use
     * @return The loaded Path object
     */
    static Path loadPath(JSONObject json, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        return buildPathFromJson(json, defaultGlobalConstraints);
    }

    /**
     * Loads a path from a JSON file in the default project root directory.
     * 
     * <p>This is equivalent to calling {@code loadPath(projectRoot(), pathFileName)}.
     * 
     * @param pathFileName The name of the path file (including .json extension)
     * @return The loaded Path object
     * @throws RuntimeException if the file cannot be read or parsed
     */
    static Path loadPath(String pathFileName) {
        return loadPath(projectRoot(), pathFileName);
    }

    /**
     * Loads a path from a JSON string with specified global constraints.
     * 
     * <p>This method is useful when the JSON data comes from a source other than a file,
     * such as network communication or embedded resources.
     * 
     * @param pathJson The JSON string representing the path
     * @param defaultGlobalConstraints The default global constraints to use
     * @return The loaded Path object
     * @throws RuntimeException if the JSON string cannot be parsed
     */
    static Path loadPathFromJsonString(String pathJson, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        try {
            JSONObject json = object(new JSONParser().parse(pathJson), "path");
            return buildPathFromJson(json, defaultGlobalConstraints);
        } catch (ParseException e) {
            throw new RuntimeException("Failed to parse path JSON string", e);
        }
    }

    /**
     * Builds a Path object from a JSON object and global constraints.
     * 
     * @param json The JSON object containing path data
     * @param defaultGlobalConstraints The default global constraints to use
     * @return The constructed Path object
     */
    private static Path buildPathFromJson(JSONObject json, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        ArrayList<PathElement> elements = parsePathElements(json);

        Path.PathConstraints constraints = parsePathConstraints(json);

        Optional<HandoffMode> handoffMode = readHandoffMode(json.get("handoff_mode"), "handoff_mode");
        DriveDirection direction = DriveDirection.FORWARD;
        if (json.containsKey("tank_drive_direction")) {
            Object raw = json.get("tank_drive_direction");
            if ("backward".equals(raw)) direction = DriveDirection.BACKWARD;
            else if (!"forward".equals(raw))
                throw new IllegalArgumentException("tank_drive_direction: expected \"forward\" or \"backward\", received " + raw);
        }
        Path.DefaultGlobalConstraints globals = defaultGlobalConstraints;
        ProjectConfig config = null;
        JSONObject globalsJson = json.get("default_global_constraints") == null ? null
            : object(json.get("default_global_constraints"), "default_global_constraints");
        if (globalsJson != null) {
            globals = parseDefaultGlobalConstraints(globalsJson);
        } else if (globals == null) {
            config = readProjectConfig(projectRoot());
            globals = config.defaults();
        }

        // Only publish shared defaults after all present fields have been parsed successfully.
        Path path = new Path(elements, constraints, globals);
        handoffMode.ifPresent(path::setHandoffMode);
        path.setTankDriveDirection(direction);
        if (config != null) Path.setDefaultHandoffMode(config.handoffMode());
        return path;
    }

    /**
     * Parses path elements from a JSON object.
     * 
     * <p>Supports three element types:
     * <ul>
     *   <li><b>translation:</b> A position target with optional handoff radius</li>
     *   <li><b>rotation:</b> A holonomic rotation target with t_ratio and optional profiling</li>
     *   <li><b>waypoint:</b> Combined translation and rotation target</li>
     * </ul>
     * 
     * @param json The JSON object containing path_elements array
     * @return ArrayList of parsed PathElement objects
     */
    private static ArrayList<PathElement> parsePathElements(JSONObject json) {
        Object raw = json.get("path_elements");
        if (!(raw instanceof JSONArray items)) {
            throw new IllegalArgumentException("path_elements must be an array");
        }
        ArrayList<PathElement> elements = new ArrayList<>(items.size());
        for (int index = 0; index < items.size(); index++) {
            String context = "element " + (index + 1);
            JSONObject element = object(items.get(index), context);
            Object type = element.get("type");
            if (!(type instanceof String)) throw new IllegalArgumentException(context + ".type must be a string");
            switch ((String) type) {
                case "translation" -> elements.add(translation(element, context + ".translation"));
                case "rotation" -> elements.add(rotation(element, context + ".rotation"));
                case "event_trigger" -> {
                    Object key = element.get("lib_key");
                    if (!(key instanceof String)) throw new IllegalArgumentException(context + ".lib_key must be a string");
                    elements.add(new EventTrigger(optionalNumber(element, "t_ratio", context).orElse(.5), (String) key));
                }
                case "waypoint" -> elements.add(new Waypoint(
                    translation(object(element.get("translation_target"), context + ".translation_target"), context + ".translation_target"),
                    rotation(object(element.get("rotation_target"), context + ".rotation_target"), context + ".rotation_target")));
                default -> throw new IllegalArgumentException(context + ".type is unsupported: " + type);
            }
        }
        return elements;
    }

    private static TranslationTarget translation(JSONObject json, String context) {
        return new TranslationTarget(new Translation2d(number(json, "x_meters", context), number(json, "y_meters", context)),
            optionalNumber(json, "intermediate_handoff_radius_meters", context),
            readHandoffMode(json.get("handoff_mode"), context + ".handoff_mode"));
    }

    private static RotationTarget rotation(JSONObject json, String context) {
        Object profiled = json.get("profiled_rotation");
        if (profiled != null && !(profiled instanceof Boolean)) {
            throw new IllegalArgumentException(context + ".profiled_rotation must be a boolean");
        }
        return new RotationTarget(Rotation2d.fromRadians(number(json, "rotation_radians", context)),
            optionalNumber(json, "t_ratio", context).orElse(.5), Boolean.TRUE.equals(profiled));
    }

    private static JSONObject object(Object value, String context) {
        if (value instanceof JSONObject json) return json;
        throw new IllegalArgumentException(context + " must be an object");
    }

    private static double number(JSONObject json, String key, String context) {
        Object value = json.get(key);
        if (value == null) throw new IllegalArgumentException(context + "." + key + " is missing");
        if (!(value instanceof Number n) || !Double.isFinite(n.doubleValue())) {
            throw new IllegalArgumentException(context + "." + key + " must be a finite number");
        }
        return n.doubleValue();
    }

    private static Optional<Double> optionalNumber(JSONObject json, String key, String context) {
        return json.get(key) == null ? Optional.empty() : Optional.of(number(json, key, context));
    }

    private static Optional<HandoffMode> readHandoffMode(Object value, String context) {
        if (value == null) return Optional.empty();
        if (value instanceof String name) {
            try { return Optional.of(HandoffMode.valueOf(name.toUpperCase(java.util.Locale.ROOT))); }
            catch (IllegalArgumentException ignored) { /* report with the field context below */ }
        }
        throw new IllegalArgumentException(context + " must be radius or progress");
    }

    /**
     * Parses path constraints from a JSON object.
     * 
     * <p>Constraints are optional and can include:
     * <ul>
     *   <li>max_velocity_meters_per_sec</li>
     *   <li>max_acceleration_meters_per_sec2</li>
     *   <li>max_velocity_deg_per_sec</li>
     *   <li>max_acceleration_deg_per_sec2</li>
     *   <li>min_velocity_meters_per_sec</li>
     *   <li>min_velocity_deg_per_sec</li>
     *   <li>end_translation_tolerance_meters</li>
     *   <li>end_rotation_tolerance_deg</li>
     * </ul>
     * 
     * @param json The JSON object containing constraints
     * @return PathConstraints object with parsed values
     */
    private static Path.PathConstraints parsePathConstraints(JSONObject json) {
        Path.PathConstraints constraints = new Path.PathConstraints();
        JSONObject constraintsJson = getNestedObject(json, "constraints");
        if (constraintsJson == null) {
            constraintsJson = findBestObjectContainingKeys(json, PATH_CONSTRAINT_KEY_ALIASES);
        }

        parseConstraint(constraintsJson, json, "max_velocity_meters_per_sec", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxVelocityMetersPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "max_acceleration_meters_per_sec2", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxAccelerationMetersPerSec2(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "max_velocity_deg_per_sec", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxVelocityDegPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "max_acceleration_deg_per_sec2", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxAccelerationDegPerSec2(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "min_velocity_meters_per_sec", (val) -> {
            if (val.isPresent()) {
                constraints.setMinVelocityMetersPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "min_velocity_deg_per_sec", (val) -> {
            if (val.isPresent()) {
                constraints.setMinVelocityDegPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });

        lookupValueByKeys(constraintsJson, json, "end_translation_tolerance_meters")
            .map(value -> finiteConstraint(value, "end_translation_tolerance_meters"))
            .ifPresent(constraints::setEndTranslationToleranceMeters);
        lookupValueByKeys(constraintsJson, json, "end_rotation_tolerance_deg")
            .map(value -> finiteConstraint(value, "end_rotation_tolerance_deg"))
            .ifPresent(constraints::setEndRotationToleranceDeg);

        return constraints;
    }

    /**
     * Parses default global constraints from a JSON object.
     * 
     * @param json The JSON object containing global constraint values
     * @return DefaultGlobalConstraints with all required values
     */
    private static Path.DefaultGlobalConstraints parseDefaultGlobalConstraints(JSONObject json) {
        JSONObject constraintsJson = getNestedObject(json, "kinematic_constraints");
        if (constraintsJson == null) {
            constraintsJson = findBestObjectContainingKeys(json, GLOBAL_CONSTRAINT_KEY_ALIASES);
        }

        double dMaxVelMps = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_velocity_meters_per_sec",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxVelocityMetersPerSec(),
            "max_velocity_meters_per_sec"
        );
        double dMaxAccMps2 = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_acceleration_meters_per_sec2",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxAccelerationMetersPerSec2(),
            "max_acceleration_meters_per_sec2"
        );
        double dMaxVelDeg = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_velocity_deg_per_sec",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxVelocityDegPerSec(),
            "max_velocity_deg_per_sec"
        );
        double dMaxAccDeg2 = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_acceleration_deg_per_sec2",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxAccelerationDegPerSec2(),
            "max_acceleration_deg_per_sec2"
        );
        double endTransTol = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_end_translation_tolerance_meters",
            FALLBACK_GLOBAL_CONSTRAINTS.getEndTranslationToleranceMeters(),
            "end_translation_tolerance_meters"
        );
        double endRotTolDeg = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_end_rotation_tolerance_deg",
            FALLBACK_GLOBAL_CONSTRAINTS.getEndRotationToleranceDeg(),
            "end_rotation_tolerance_deg"
        );
        double handoffRadius = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_intermediate_handoff_radius_meters",
            FALLBACK_GLOBAL_CONSTRAINTS.getIntermediateHandoffRadiusMeters(),
            "intermediate_handoff_radius_meters"
        );

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

    /**
     * Parses a ranged constraint array from JSON.
     * 
     * @param constraintsJson The constraints JSON object
     * @param key The key for the constraint array
     * @param setter Consumer to set the parsed constraint values
     */
    private static void parseConstraint(
        JSONObject constraintsJson,
        JSONObject rootJson,
        String key,
        Consumer<Optional<ArrayList<Path.RangedConstraint>>> setter
    ) {
        Optional<Object> arrObj = lookupValueByKeys(constraintsJson, rootJson, key);
        if (arrObj.isEmpty()) return;
        if (!(arrObj.get() instanceof JSONArray arr))
            throw new IllegalArgumentException("constraints." + key + " must be an array");

        ArrayList<Path.RangedConstraint> list = new ArrayList<>();
        for (int index = 0; index < arr.size(); index++) {
            String context = "constraints." + key + "[" + index + "]";
            JSONObject rcJson = object(arr.get(index), context);
            Optional<Double> value = toDouble(rcJson.get("value"));
            Optional<Integer> startOrdinal = toInt(rcJson.get("start_ordinal"));
            Optional<Integer> endOrdinal = toInt(rcJson.get("end_ordinal"));
            if (value.isEmpty() || !Double.isFinite(value.get()))
                throw new IllegalArgumentException(context + ".value must be a finite number");
            if (startOrdinal.isEmpty())
                throw new IllegalArgumentException(context + ".start_ordinal must be an integer");
            if (endOrdinal.isEmpty())
                throw new IllegalArgumentException(context + ".end_ordinal must be an integer");
            list.add(new Path.RangedConstraint(value.get(), startOrdinal.get(), endOrdinal.get()));
        }
        if (!list.isEmpty()) {
            setter.accept(Optional.of(list));
        }
    }

    /**
     * Loads global constraints from a config.json file in the specified directory.
     * 
     * <p>The config.json file should contain default values for all constraint types:
     * <ul>
     *   <li>default_max_velocity_meters_per_sec</li>
     *   <li>default_max_acceleration_meters_per_sec2</li>
     *   <li>default_max_velocity_deg_per_sec</li>
     *   <li>default_max_acceleration_deg_per_sec2</li>
     *   <li>default_end_translation_tolerance_meters</li>
     *   <li>default_end_rotation_tolerance_deg</li>
     *   <li>default_intermediate_handoff_radius_meters</li>
     * </ul>
     * 
     * @param autosDir The directory containing config.json
     * @return DefaultGlobalConstraints loaded from the config file
     * @throws RuntimeException if the config file cannot be read or parsed
     */
    static Path.DefaultGlobalConstraints loadGlobalConstraints(File autosDir) {
        ProjectConfig config = readProjectConfig(autosDir);
        Path.setDefaultHandoffMode(config.handoffMode());
        return config.defaults();
    }

    private record ProjectConfig(Path.DefaultGlobalConstraints defaults, HandoffMode handoffMode) {}

    private static ProjectConfig readProjectConfig(File autosDir) {
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

            JSONObject json = object(new JSONParser().parse(fileContent), "config");
            Path.DefaultGlobalConstraints defaults = parseDefaultGlobalConstraints(json);
            HandoffMode mode = readHandoffMode(lookupValueByKeys(
                getNestedObject(json, "kinematic_constraints"), json, "default_handoff_mode")
                .orElse(null), "default_handoff_mode").orElse(HandoffMode.RADIUS);
            return new ProjectConfig(defaults, mode);
        } catch (IOException | ParseException | RuntimeException e) {
            throw new IllegalArgumentException("config.json: " + e.getMessage(), e);
        }
    }

    private static JSONObject getNestedObject(JSONObject json, String key) {
        Object val = json.get(key);
        return val instanceof JSONObject ? (JSONObject) val : null;
    }

    private static Optional<Object> lookupValueByKeys(
        JSONObject preferredContainer,
        JSONObject rootJson,
        String... keys
    ) {
        for (String key : keys) {
            Optional<Object> preferred = lookupDirect(preferredContainer, key);
            if (preferred.isPresent()) {
                return preferred;
            }
        }
        for (String key : keys) {
            Optional<Object> direct = lookupDirect(rootJson, key);
            if (direct.isPresent()) {
                return direct;
            }
        }
        for (String key : keys) {
            Optional<Object> recursive = findFirstValueByKey(rootJson, key);
            if (recursive.isPresent()) {
                return recursive;
            }
        }
        return Optional.empty();
    }

    private static Optional<Object> lookupDirect(JSONObject json, String key) {
        if (json == null || !json.containsKey(key)) {
            return Optional.empty();
        }
        Object value = json.get(key);
        return value == null ? Optional.empty() : Optional.of(value);
    }

    private static Optional<Object> findFirstValueByKey(Object node, String key) {
        if (node instanceof JSONObject obj) {
            if (obj.containsKey(key)) {
                Object value = obj.get(key);
                if (value != null) {
                    return Optional.of(value);
                }
            }
            for (Object child : obj.values()) {
                Optional<Object> found = findFirstValueByKey(child, key);
                if (found.isPresent()) {
                    return found;
                }
            }
        } else if (node instanceof JSONArray arr) {
            for (Object child : arr) {
                Optional<Object> found = findFirstValueByKey(child, key);
                if (found.isPresent()) {
                    return found;
                }
            }
        }
        return Optional.empty();
    }

    private static JSONObject findBestObjectContainingKeys(JSONObject rootJson, String[][] keyAliases) {
        ArrayList<JSONObject> objects = new ArrayList<>();
        collectJsonObjects(rootJson, objects);

        JSONObject best = null;
        int bestScore = 0;
        for (JSONObject candidate : objects) {
            int score = countMatchingKeys(candidate, keyAliases);
            if (score > bestScore) {
                bestScore = score;
                best = candidate;
            }
        }
        return best;
    }

    private static void collectJsonObjects(Object node, ArrayList<JSONObject> out) {
        if (node instanceof JSONObject obj) {
            out.add(obj);
            for (Object child : obj.values()) {
                collectJsonObjects(child, out);
            }
        } else if (node instanceof JSONArray arr) {
            for (Object child : arr) {
                collectJsonObjects(child, out);
            }
        }
    }

    private static int countMatchingKeys(JSONObject json, String[][] keyAliases) {
        int score = 0;
        for (String[] aliases : keyAliases) {
            for (String alias : aliases) {
                if (json.containsKey(alias) && json.get(alias) != null) {
                    score++;
                    break;
                }
            }
        }
        return score;
    }

    private static Optional<Double> toDouble(Object value) {
        if (value instanceof Number number) {
            return Optional.of(number.doubleValue());
        }
        if (value instanceof String text) {
            try {
                return Optional.of(Double.parseDouble(text.trim()));
            } catch (NumberFormatException ignored) {
                return Optional.empty();
            }
        }
        return Optional.empty();
    }

    private static Optional<Integer> toInt(Object value) {
        if (value instanceof Number number) {
            double ordinal = number.doubleValue();
            if (!Double.isFinite(ordinal) || ordinal != Math.rint(ordinal)
                || ordinal < Integer.MIN_VALUE || ordinal > Integer.MAX_VALUE) return Optional.empty();
            return Optional.of((int) ordinal);
        }
        if (value instanceof String text) {
            try {
                return Optional.of(Integer.parseInt(text.trim()));
            } catch (NumberFormatException ignored) {
                return Optional.empty();
            }
        }
        return Optional.empty();
    }

    private static double finiteConstraint(Object value, String field) {
        var parsed = toDouble(value);
        if (parsed.isEmpty() || !Double.isFinite(parsed.get()))
            throw new IllegalArgumentException("constraints." + field + " must be a finite number");
        return parsed.get();
    }

    private static double readDoubleConstraintValue(
        JSONObject preferredContainer,
        JSONObject rootJson,
        String primaryKey,
        double fallbackValue,
        String... aliases
    ) {
        String[] keys = new String[aliases.length + 1];
        keys[0] = primaryKey;
        System.arraycopy(aliases, 0, keys, 1, aliases.length);

        Optional<Object> raw = lookupValueByKeys(preferredContainer, rootJson, keys);
        if (raw.isEmpty()) {
            System.err.println(
                "BLine JsonUtils: Missing constraint key '" + primaryKey + "', using fallback value " + fallbackValue
            );
            return fallbackValue;
        }
        Optional<Double> parsed = toDouble(raw.get());
        if (parsed.isEmpty() || !Double.isFinite(parsed.get())) {
            throw new IllegalArgumentException(primaryKey + " must be a finite number");
        }
        return parsed.get();
    }

}

package frc.robot.lib.BLine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.WeakHashMap;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.smartdashboard.Field2d;

/**
 * Helpers for visualizing BLine paths on a WPILib {@link Field2d} widget.
 *
 * <p>BLine paths are polylines, so a path can be displayed directly from
 * {@link Path#getTranslations()}.
 *
 * <pre>{@code
 * Field2d field = new Field2d();
 * // Periodically publish using the robot's WPILib TelemetryTable:
 * table.log("Field", field);
 *
 * BLineField.drawPath(field, scoreTwoPath);
 * BLineField.drawPath(field, "Leave", leavePath);
 * }</pre>
 *
 * <h2>How it renders (and why object names end in "Trajectory")</h2>
 * <p>Each {@code FieldObject2d} is published as a list of poses. Elastic decides
 * whether to draw an object as a <b>connected polyline</b> or as <b>individual
 * pose markers</b> based on the object's name and size: an object is drawn as a
 * connected line when its NetworkTables name ends with {@code "trajectory"}
 * (case-insensitive) or when it contains more than 8 poses; otherwise it is drawn
 * as discrete arrow markers, one per pose.
 *
 * <p>These helpers ensure object names end in {@code "Trajectory"} so short
 * paths render as clean polylines in Elastic. Field2d requires poses, so these
 * helpers create display-only rotations from the polyline direction when
 * drawing.
 */
public final class BLineField {
    private static final double POINT_EPSILON = 1e-6;
    private static final String AUTO_PATH_OBJECT_PREFIX = "BLinePath";
    private static final Map<Field2d, Map<Path, String>> AUTO_PATH_OBJECT_NAMES =
        Collections.synchronizedMap(new WeakHashMap<>());
    private static final Map<Field2d, Integer> AUTO_OBJECT_INDICES =
        Collections.synchronizedMap(new WeakHashMap<>());

    private BLineField() {
        throw new UnsupportedOperationException("BLineField is a utility class and cannot be instantiated");
    }

    /**
     * Draws a path's planned polyline using a stable, generated object name.
     *
     * <p>The generated name is unique per {@link Field2d} and {@link Path}
     * instance, and repeated calls with the same field and path reuse the same
     * object name.
     *
     * @param field The field widget to draw on
     * @param path The path to draw
     * @return The final {@link Field2d} object name used
     */
    public static String drawPath(Field2d field, Path path) {
        Objects.requireNonNull(field, "field cannot be null");
        Objects.requireNonNull(path, "path cannot be null");

        String objectName = getOrCreateAutoObjectName(field, path);
        return drawPath(field, objectName, path);
    }

    /**
     * Draws a path's planned polyline using the given object name.
     *
     * <p>The object name is trimmed, and {@code "Trajectory"} is appended if the
     * name does not already end with that suffix case-insensitively. Reusing the
     * same explicit name updates the same display slot by design.
     *
     * @param field The field widget to draw on
     * @param objectName The {@link Field2d} object name or base name to publish under
     * @param path The path to draw
     * @return The final {@link Field2d} object name used
     */
    public static String drawPath(Field2d field, String objectName, Path path) {
        Objects.requireNonNull(field, "field cannot be null");
        Objects.requireNonNull(objectName, "objectName cannot be null");
        Objects.requireNonNull(path, "path cannot be null");

        String trajectoryObjectName = toTrajectoryObjectName(objectName);
        field.getObject(trajectoryObjectName).setPoses(toPoses(path.getTranslations()));
        return trajectoryObjectName;
    }

    private static String getOrCreateAutoObjectName(Field2d field, Path path) {
        synchronized (AUTO_PATH_OBJECT_NAMES) {
            Map<Path, String> fieldObjectNames =
                AUTO_PATH_OBJECT_NAMES.computeIfAbsent(field, ignored -> new WeakHashMap<>());
            String existingName = fieldObjectNames.get(path);
            if (existingName != null) {
                return existingName;
            }

            int nextIndex = AUTO_OBJECT_INDICES.getOrDefault(field, 0);
            String objectName = AUTO_PATH_OBJECT_PREFIX + nextIndex + "Trajectory";
            AUTO_OBJECT_INDICES.put(field, nextIndex + 1);
            fieldObjectNames.put(path, objectName);
            return objectName;
        }
    }

    /**
     * Converts an ordered list of polyline points into poses for {@link Field2d}.
     */
    private static List<Pose2d> toPoses(List<Translation2d> points) {
        List<Pose2d> poses = new ArrayList<>();
        if (points.isEmpty()) {
            return poses;
        }

        int n = points.size();
        Rotation2d previousRotation = Rotation2d.ZERO;
        for (int i = 0; i < n; i++) {
            Translation2d current = points.get(i);
            Translation2d next = findNextDistinctPoint(points, i);

            Rotation2d rotation = next == null
                ? previousRotation
                : new Rotation2d(next.getX() - current.getX(), next.getY() - current.getY());

            poses.add(new Pose2d(current, rotation));
            previousRotation = rotation;
        }

        return poses;
    }

    private static Translation2d findNextDistinctPoint(List<Translation2d> points, int currentIndex) {
        Translation2d current = points.get(currentIndex);
        for (int i = currentIndex + 1; i < points.size(); i++) {
            Translation2d candidate = points.get(i);
            if (current.getDistance(candidate) > POINT_EPSILON) {
                return candidate;
            }
        }
        return null;
    }

    private static String toTrajectoryObjectName(String objectName) {
        String trimmed = objectName.trim();
        if (trimmed.isEmpty()) {
            throw new IllegalArgumentException("objectName cannot be blank");
        }
        if (trimmed.toLowerCase(Locale.ROOT).endsWith("trajectory")) {
            return trimmed;
        }
        return trimmed + "Trajectory";
    }
}

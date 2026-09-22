package frc.robot.lib.BLine.following;

import frc.robot.lib.BLine.path.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Keeps the requested heading continuous when translation hands off early.
 * At a cut corner, the remaining interpolation starts from the heading requested at
 * the same robot position on the old segment. PID and slew limits control actual motion.
 * Progress is measured along the path in metres; headings are unwrapped radians.
 */
final class RotationProgress {
    private static final double EPSILON = 1e-9;

    private record Segment(int targetIndex, Translation2d start, Translation2d end, double startS, double length) {
        double ratio(Translation2d point) {
            if (length < EPSILON) return 1;
            Translation2d delta = point.minus(start);
            Translation2d direction = end.minus(start);
            return Math.clamp((delta.getX() * direction.getX() + delta.getY() * direction.getY()) / (length * length), 0, 1);
        }
        double project(Translation2d point) { return startS + length * ratio(point); }
        double distance(Translation2d point) { return start.interpolate(end, ratio(point)).getDistance(point); }
        double along(Translation2d movement) {
            Translation2d direction = end.minus(start);
            return movement.getX() * direction.getX() + movement.getY() * direction.getY();
        }
    }

    private record Frame(int elementIndex, double s, double theta, boolean interpolate) {}
    private record Join(double s, double theta, double endS, double endTheta) {}
    record Sample(double headingRadians, int activeIndex, int previousIndex, double intervalProgress) {}

    private final List<Segment> segments = new ArrayList<>();
    private final List<Frame> frames = new ArrayList<>();
    private final double initialHeading;
    private Translation2d previousPosition;
    private int segmentIndex;
    private double acceptedS;
    private Join join;
    private Double tailHeading;

    RotationProgress(List<Path.PathElement> elements, Pose2d startPose) {
        initialHeading = startPose.getRotation().getRadians();
        previousPosition = startPose.getTranslation();
        Translation2d previous = previousPosition;
        double s = 0;
        for (int i = 0; i < elements.size(); i++) {
            if (elements.get(i) instanceof Path.TranslationTarget target) {
                double length = previous.getDistance(target.translation());
                segments.add(new Segment(i, previous, target.translation(), s, length));
                s += length;
                previous = target.translation();
            }
        }
        for (int i = 0; i < elements.size(); i++) {
            if (!(elements.get(i) instanceof Path.RotationTarget rotation)) continue;
            Segment owning = null;
            for (Segment segment : segments) {
                if (segment.targetIndex > i) { owning = segment; break; }
            }
            if (owning != null) {
                frames.add(new Frame(i, owning.startS + Math.clamp(rotation.t_ratio(), 0, 1) * owning.length,
                    rotation.rotation().getRadians(), rotation.profiledRotation()));
            }
        }
        // Equal-distance authored targets deliberately resolve to the last target in path order.
        frames.sort(Comparator.comparingDouble(Frame::s));
        double unwrapped = initialHeading;
        for (int i = 0; i < frames.size(); i++) {
            Frame frame = frames.get(i);
            unwrapped += MathUtil.angleModulus(frame.theta - unwrapped);
            frames.set(i, new Frame(frame.elementIndex, frame.s, unwrapped, frame.interpolate));
        }
    }

    Sample update(Translation2d position, int authorizedTranslationIndex) {
        if (segments.isEmpty()) return new Sample(initialHeading, -1, -1, 0);
        while (segmentIndex + 1 < segments.size() && segments.get(segmentIndex).length < EPSILON
            && segments.get(segmentIndex + 1).targetIndex <= authorizedTranslationIndex) segmentIndex++;
        Segment oldSegment = segments.get(segmentIndex);
        double oldS = Math.max(acceptedS, oldSegment.project(position));
        double oldHeading = heading(oldS);
        Translation2d movement = position.minus(previousPosition);
        boolean switched = false;
        while (segmentIndex + 1 < segments.size()) {
            Segment current = segments.get(segmentIndex);
            Segment next = segments.get(segmentIndex + 1);
            if (next.targetIndex > authorizedTranslationIndex) break;
            // Coincident/retraced legs need traversal and motion direction to break distance ties.
            boolean retracing = Math.abs(current.distance(position) - next.distance(position)) <= EPSILON
                && current.along(movement) < -EPSILON && next.along(movement) > EPSILON;
            if (current.length < EPSILON || current.ratio(position) >= 1 - EPSILON
                || next.distance(position) + EPSILON < current.distance(position) || retracing) {
                segmentIndex++;
                switched = true;
            } else break;
        }
        acceptedS = Math.max(oldS, segments.get(segmentIndex).project(position));
        if (switched && acceptedS > oldS + EPSILON) {
            Frame future = nextFrame(acceptedS);
            // Direct-heading targets intentionally remain immediate, as in existing files.
            if (future != null && future.interpolate && Math.abs(heading(acceptedS) - oldHeading) > EPSILON) {
                join = new Join(acceptedS, oldHeading, future.s, future.theta);
                tailHeading = null;
            } else if (future == null && Math.abs(heading(acceptedS) - oldHeading) > EPSILON) {
                Segment segment = segments.get(segmentIndex);
                double endS = segment.startS + segment.length;
                if (endS > acceptedS + EPSILON) {
                    join = new Join(acceptedS, oldHeading, endS, finalHeadingRadians());
                    tailHeading = null;
                } else tailHeading = oldHeading;
            }
        }
        previousPosition = position;
        Frame next = nextFrame(acceptedS);
        Frame previous = previousFrame(acceptedS);
        double startS = previous == null ? 0 : previous.s;
        double ratio = next == null ? 1 : Math.clamp((acceptedS - startS) / Math.max(EPSILON, next.s - startS), 0, 1);
        return new Sample(heading(acceptedS), next == null ? -1 : next.elementIndex,
            previous == null ? -1 : previous.elementIndex, ratio);
    }

    double finalHeadingRadians() { return frames.isEmpty() ? initialHeading : frames.getLast().theta; }
    int finalElementIndex() { return frames.isEmpty() ? -1 : frames.getLast().elementIndex; }

    Pose2d targetPose(int elementIndex) {
        for (Frame frame : frames) {
            if (frame.elementIndex != elementIndex) continue;
            for (Segment segment : segments) {
                if (segment.startS + segment.length + EPSILON < frame.s) continue;
                double ratio = segment.length < EPSILON ? 1 : Math.clamp((frame.s - segment.startS) / segment.length, 0, 1);
                return new Pose2d(segment.start.interpolate(segment.end, ratio), new org.wpilib.math.geometry.Rotation2d(frame.theta));
            }
        }
        throw new IllegalArgumentException("Unknown rotation element " + elementIndex);
    }

    private Frame nextFrame(double s) {
        for (Frame frame : frames) if (frame.s > s + EPSILON) return frame;
        return null;
    }

    private Frame previousFrame(double s) {
        Frame previous = null;
        for (Frame frame : frames) {
            if (frame.s > s + EPSILON) break;
            previous = frame;
        }
        return previous;
    }

    private double heading(double s) {
        Frame next = nextFrame(s);
        if (next != null && !next.interpolate) return next.theta;
        if (join != null && s >= join.s && s < join.endS) {
            return join.theta + (s - join.s) / (join.endS - join.s) * (join.endTheta - join.theta);
        }
        if (next == null) return tailHeading == null ? finalHeadingRadians() : tailHeading;
        Frame previous = previousFrame(s);
        double startS = previous == null ? 0 : previous.s;
        double startHeading = previous == null ? initialHeading : previous.theta;
        return startHeading + Math.clamp((s - startS) / (next.s - startS), 0, 1) * (next.theta - startHeading);
    }
}

package frc.robot.lib.BLine.following;

import frc.robot.lib.BLine.path.HandoffMode;
import frc.robot.lib.BLine.path.Path.*;
import frc.robot.lib.BLine.path.PreparedPath.MotionConstraint;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.util.Pair;

/** Tracks ordered translation targets and projects the robot onto their incoming segments. */
final class TranslationProgress {
    private static final double SEGMENT_LENGTH_EPSILON_METERS = 1e-6;
    // Retains the existing projection fallback for segments shorter than 1 mm.
    private static final double PROJECTION_LENGTH_SQUARED_EPSILON = 1e-6;
    record Segment(Translation2d startTranslation, Translation2d endTranslation,
                   double segmentLength, double segmentProgress) {
        boolean isDegenerate() { return segmentLength < SEGMENT_LENGTH_EPSILON_METERS; }
    }
    record CrossTrack(Translation2d closestPoint, double errorMeters) {}

    private final List<Pair<PathElement, MotionConstraint>> elements;
    private final Translation2d origin;
    private int index;

    TranslationProgress(List<Pair<PathElement, MotionConstraint>> elements, Pose2d start) {
        this.elements = elements;
        origin = start.getTranslation();
        index = next(0);
    }

    int index() { return index; }
    boolean isLast() { return next(index + 1) < 0; }
    Translation2d target() { return translation(index); }

    void advance(Pose2d pose) {
        while (!isLast()) {
            TranslationTarget target = (TranslationTarget) elements.get(index).getFirst();
            Segment segment = segment(pose);
            double distance = pose.getTranslation().getDistance(target.translation());
            double handoff = target.intermediateHandoffRadiusMeters().orElseThrow();
            double threshold = segment.isDegenerate() ? 0 : Math.clamp(1 - handoff / segment.segmentLength(), 0, 1);
            boolean reached = segment.isDegenerate() || distance <= handoff
                || target.handoffMode().orElseThrow() == HandoffMode.PROGRESS && segment.segmentProgress() >= threshold;
            if (!reached) return;
            index = next(index + 1);
        }
    }

    Segment segment(Pose2d pose) {
        Translation2d start = translation(previous(index - 1));
        Translation2d end = target();
        double length = start.getDistance(end);
        return new Segment(start, end, length, length < SEGMENT_LENGTH_EPSILON_METERS
            ? 1 : project(start, end, pose.getTranslation()));
    }

    double remainingDistance(Pose2d pose) {
        Translation2d previous = pose.getTranslation();
        double remaining = 0;
        for (int i = index; i >= 0; i = next(i + 1)) {
            Translation2d target = translation(i);
            remaining += previous.getDistance(target);
            previous = target;
        }
        return remaining;
    }

    /** Positive error is left of the directed segment; collinear overshoot is not lateral error. */
    CrossTrack crossTrack(Pose2d pose) {
        Translation2d start = translation(previous(index - 1));
        Translation2d end = target();
        Translation2d position = pose.getTranslation();
        double t = project(start, end, position);
        Translation2d closest = start.interpolate(end, t);
        double cross = (end.getX() - start.getX()) * (position.getY() - start.getY())
            - (end.getY() - start.getY()) * (position.getX() - start.getX());
        double error = Math.abs(cross) <= SEGMENT_LENGTH_EPSILON_METERS * start.getDistance(end)
            ? 0 : Math.copySign(position.getDistance(closest), cross);
        return new CrossTrack(closest, error);
    }

    boolean eventReached(int eventIndex, Pose2d pose) {
        if (eventIndex > index) return false;
        int endIndex = next(eventIndex + 1);
        if (endIndex < index) return true;
        if (endIndex < 0) return false;
        Translation2d start = translation(previous(eventIndex - 1));
        Translation2d end = translation(endIndex);
        if (start.getDistance(end) < SEGMENT_LENGTH_EPSILON_METERS) return true;
        EventTrigger event = (EventTrigger) elements.get(eventIndex).getFirst();
        return project(start, end, pose.getTranslation()) >= event.t_ratio();
    }

    private int next(int start) {
        for (int i = Math.max(0, start); i < elements.size(); i++)
            if (elements.get(i).getFirst() instanceof TranslationTarget) return i;
        return -1;
    }
    private int previous(int start) {
        for (int i = start; i >= 0; i--)
            if (elements.get(i).getFirst() instanceof TranslationTarget) return i;
        return -1;
    }
    private Translation2d translation(int index) {
        return index < 0 ? origin : ((TranslationTarget) elements.get(index).getFirst()).translation();
    }
    private static double project(Translation2d start, Translation2d end, Translation2d point) {
        double dx = end.getX() - start.getX(), dy = end.getY() - start.getY();
        double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared < PROJECTION_LENGTH_SQUARED_EPSILON) return 0;
        return Math.clamp(((point.getX() - start.getX()) * dx + (point.getY() - start.getY()) * dy) / lengthSquared, 0, 1);
    }
}

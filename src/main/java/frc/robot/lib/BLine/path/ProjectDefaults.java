package frc.robot.lib.BLine.path;

import java.util.Objects;

/**
 * Immutable project defaults: numerical constraints and the fallback handoff mode.
 * Loading this value does not change any path. Install it with {@link Path#setProjectDefaults}
 * to affect subsequent executions; active followers keep their existing snapshots.
 *
 * <pre>{@code
 * var defaults = Path.loadProjectDefaults(autosDirectory);
 * Path.setProjectDefaults(defaults);
 * }</pre>
 *
 * @param constraints numerical defaults shared by paths without explicit overrides
 * @param handoffMode fallback when neither the element nor path specifies a mode
 */
public record ProjectDefaults(Path.DefaultGlobalConstraints constraints, HandoffMode handoffMode) {
    public ProjectDefaults {
        Objects.requireNonNull(constraints, "constraints");
        Objects.requireNonNull(handoffMode, "handoffMode");
    }
}

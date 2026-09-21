package frc.robot.lib.BLine;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

/** Scheduler-thread queue: dispatch occurs outside the path coroutine, so events have independent lifetimes. */
final class PendingEvents {
    private static final Logger logger = Logger.getLogger(PendingEvents.class.getName());
    private final Map<String, Runnable> registry = new HashMap<>();
    private final ArrayDeque<Entry> pending = new ArrayDeque<>();
    private long clearGeneration;

    static final class Execution {
        private boolean cancelled;
    }

    private record Entry(Execution owner, Runnable action) {}

    void register(String key, Runnable action) {
        if (key == null || key.isBlank() || action == null) {
            logger.warning("FollowPath: Ignoring invalid event trigger registration");
            return;
        }
        registry.put(key, action);
    }

    void enqueue(Execution owner, String key) {
        Runnable action = registry.get(key);
        if (action == null) {
            logger.warning("FollowPath: Unregistered event trigger key: " + key);
        } else if (!owner.cancelled) {
            pending.addLast(new Entry(owner, action));
        }
    }

    void cancel(Execution owner) {
        if (owner == null) return;
        owner.cancelled = true;
        pending.removeIf(entry -> entry.owner() == owner);
    }

    void clear() {
        pending.clear();
        clearGeneration++;
    }

    void dispatch() {
        // Snapshot the batch: callbacks may enqueue more events, which belong to the next poll.
        long generation = clearGeneration;
        var batch = new ArrayDeque<>(pending);
        pending.clear();
        for (Entry entry : batch) {
            // A cleanup action may clear events during this poll, including this detached batch.
            if (generation != clearGeneration) break;
            if (entry.owner().cancelled) continue;
            try {
                entry.action().run();
            } catch (RuntimeException error) {
                logger.log(Level.WARNING, "FollowPath: Event trigger failed", error);
            }
        }
    }
}

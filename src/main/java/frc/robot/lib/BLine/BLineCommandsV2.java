package frc.robot.lib.BLine;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Command composition helpers that proxy child commands before delegating to WPILib.
 *
 * <p>This class mirrors the WPILib {@link Commands} methods that accept child
 * {@link Command Commands}. Each supplied child command is wrapped with
 * {@link Command#asProxy()} before it is added to the WPILib composition. That
 * keeps the returned composition from owning all child command requirements for
 * its whole lifetime, while still letting WPILib schedule each child command
 * normally when that child actually runs.
 *
 * <p>This is useful when a path contains event triggers:
 *
     * <pre>{@code
     * import static frc.robot.lib.BLine.BLineCommandsV2.sequence;
     * import org.wpilib.command2.Command;
     *
     * Command auto = sequence(
 *     shooter.shoot().withTimeout(2.0),
 *     pathing.followPath("intakethroughdepot"),
 *     shooter.shoot()
 * );
 * }</pre>
 *
 * <p>Real requirement conflicts still exist. If two scheduled commands require
 * the same subsystem at the same time, WPILib's {@link org.wpilib.command2.CommandScheduler}
 * resolves that conflict normally.
 *
 * @see Commands
 * @see Command#asProxy()
 */
public final class BLineCommandsV2 {
    private BLineCommandsV2() {
        throw new UnsupportedOperationException("This is a utility class");
    }

    /**
     * Marker-safe counterpart to {@link Commands#either(Command, Command, BooleanSupplier)}.
     *
     * <p>The supplied branch commands are wrapped with {@link Command#asProxy()}
     * before they are passed to WPILib. The returned conditional command does not
     * inherit the proxied branch requirements; those requirements are claimed
     * only when WPILib schedules the selected proxied command.
     *
     * @param onTrue the command selected when {@code selector} returns true
     * @param onFalse the command selected when {@code selector} returns false
     * @param selector selects which command branch to run
     * @return a marker-safe conditional command
     * @see Commands#either(Command, Command, BooleanSupplier)
     * @see Command#asProxy()
     */
    public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
        return Commands.either(proxy(onTrue), proxy(onFalse), selector);
    }

    /**
     * Marker-safe counterpart to {@link Commands#select(Map, Supplier)}.
     *
     * <p>Each command in {@code commands} is wrapped with {@link Command#asProxy()}
     * before it is passed to WPILib. The returned select command does not inherit
     * the proxied branch requirements; those requirements are claimed only when
     * WPILib schedules the selected proxied command.
     *
     * @param commands map of selector values to commands
     * @param selector supplies the selector value at runtime
     * @param <K> selector key type
     * @return a marker-safe select command
     * @see Commands#select(Map, Supplier)
     * @see Command#asProxy()
     */
    public static <K> Command select(Map<K, Command> commands, Supplier<? extends K> selector) {
        return Commands.select(proxyMap(commands), selector);
    }

    /**
     * Marker-safe counterpart to {@link Commands#defer(Supplier, Set)}.
     *
     * <p>The command supplied at runtime is wrapped with {@link Command#asProxy()}
     * before it is run by WPILib. The explicit {@code requirements} argument still
     * belongs to the returned deferred command, matching WPILib's deferred-command
     * contract. The supplied command's own requirements are claimed only when
     * WPILib schedules the proxied command.
     *
     * @param supplier supplies the command to proxy and run when initialized
     * @param requirements requirements for the returned deferred command
     * @return a marker-safe deferred command
     * @see Commands#defer(Supplier, Set)
     * @see Command#asProxy()
     */
    public static Command defer(Supplier<Command> supplier, Set<Subsystem> requirements) {
        Objects.requireNonNull(supplier, "supplier");
        return Commands.defer(() -> {
            Command command = supplier.get();
            return command == null ? null : command.asProxy();
        }, requirements);
    }

    /**
     * Marker-safe counterpart to {@link Commands#deferredProxy(Supplier)}.
     *
     * <p>The command supplied at runtime is wrapped with {@link Command#asProxy()}
     * before it is run by WPILib. The returned deferred proxy has no requirements;
     * the supplied command's requirements are claimed only when WPILib schedules
     * the proxied command.
     *
     * @param supplier supplies the command to proxy and run when initialized
     * @return a marker-safe deferred proxy command
     * @see Commands#deferredProxy(Supplier)
     * @see Command#asProxy()
     */
    public static Command deferredProxy(Supplier<Command> supplier) {
        return Commands.deferredProxy(supplier);
    }

    /**
     * Marker-safe counterpart to {@link Commands#sequence(Command...)}.
     *
     * <p>Each supplied command is wrapped with {@link Command#asProxy()} before it
     * is passed to WPILib. The returned sequence does not inherit the proxied
     * child requirements; those requirements are claimed only when WPILib schedules
     * each proxied command.
     *
     * @param commands commands to run in sequence
     * @return a marker-safe sequential command group
     * @see Commands#sequence(Command...)
     * @see Command#asProxy()
     */
    public static Command sequence(Command... commands) {
        return Commands.sequence(proxyAll(commands));
    }

    /**
     * Marker-safe counterpart to {@link Commands#repeatingSequence(Command...)}.
     *
     * <p>Each supplied command is wrapped with {@link Command#asProxy()} before it
     * is passed to WPILib. The returned repeating sequence does not inherit the
     * proxied child requirements; those requirements are claimed only when WPILib
     * schedules each proxied command.
     *
     * @param commands commands to run in sequence repeatedly
     * @return a marker-safe repeating sequential command group
     * @see Commands#repeatingSequence(Command...)
     * @see Command#asProxy()
     */
    public static Command repeatingSequence(Command... commands) {
        return Commands.repeatingSequence(proxyAll(commands));
    }

    /**
     * Marker-safe counterpart to {@link Commands#parallel(Command...)}.
     *
     * <p>Each supplied command is wrapped with {@link Command#asProxy()} before it
     * is passed to WPILib. The returned parallel group does not inherit the
     * proxied child requirements; those requirements are claimed only when WPILib
     * schedules each proxied command.
     *
     * @param commands commands to run in parallel
     * @return a marker-safe parallel command group
     * @see Commands#parallel(Command...)
     * @see Command#asProxy()
     */
    public static Command parallel(Command... commands) {
        return Commands.parallel(proxyAll(commands));
    }

    /**
     * Marker-safe counterpart to {@link Commands#race(Command...)}.
     *
     * <p>Each supplied command is wrapped with {@link Command#asProxy()} before it
     * is passed to WPILib. The returned race group does not inherit the proxied
     * child requirements; those requirements are claimed only when WPILib schedules
     * each proxied command.
     *
     * @param commands commands to run until the first command finishes
     * @return a marker-safe parallel race group
     * @see Commands#race(Command...)
     * @see Command#asProxy()
     */
    public static Command race(Command... commands) {
        return Commands.race(proxyAll(commands));
    }

    /**
     * Marker-safe counterpart to {@link Commands#deadline(Command, Command...)}.
     *
     * <p>The deadline and parallel commands are wrapped with {@link Command#asProxy()}
     * before they are passed to WPILib. The returned deadline group does not inherit
     * the proxied child requirements; those requirements are claimed only when
     * WPILib schedules each proxied command.
     *
     * @param deadline the command that ends the group when it finishes
     * @param commands other commands to run with the deadline
     * @return a marker-safe parallel deadline group
     * @see Commands#deadline(Command, Command...)
     * @see Command#asProxy()
     */
    public static Command deadline(Command deadline, Command... commands) {
        return Commands.deadline(proxy(deadline), proxyAll(commands));
    }

    private static Command proxy(Command command) {
        return Objects.requireNonNull(command, "command").asProxy();
    }

    private static Command[] proxyAll(Command... commands) {
        Objects.requireNonNull(commands, "commands");
        Command[] proxiedCommands = new Command[commands.length];
        for (int i = 0; i < commands.length; i++) {
            proxiedCommands[i] = proxy(commands[i]);
        }
        return proxiedCommands;
    }

    private static <K> Map<K, Command> proxyMap(Map<K, Command> commands) {
        Objects.requireNonNull(commands, "commands");
        Map<K, Command> proxiedCommands = new LinkedHashMap<>();
        for (Map.Entry<K, Command> entry : commands.entrySet()) {
            proxiedCommands.put(entry.getKey(), proxy(entry.getValue()));
        }
        return proxiedCommands;
    }
}

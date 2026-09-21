package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class BLineCommandsV2Test {
    private CommandScheduler scheduler;

    @AfterEach
    void tearDown() {
        if (scheduler != null) {
            scheduler.cancelAll();
            scheduler.clearComposedCommands();
            scheduler.enable();
            scheduler = null;
        }
    }

    @Test
    void sequenceDoesNotInheritChildRequirements() {
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.sequence(requiringCommand(subsystem));

        assertTrue(command.getRequirements().isEmpty(), "Sequence should only see proxy requirements");
    }

    @Test
    void repeatingSequenceDoesNotInheritChildRequirements() {
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.repeatingSequence(requiringCommand(subsystem));

        assertTrue(command.getRequirements().isEmpty(), "Repeating sequence should only see proxy requirements");
    }

    @Test
    void parallelDoesNotInheritChildRequirements() {
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.parallel(requiringCommand(subsystem));

        assertTrue(command.getRequirements().isEmpty(), "Parallel group should only see proxy requirements");
    }

    @Test
    void parallelAcceptsChildrenWithSameRequirement() {
        TestSubsystem subsystem = new TestSubsystem();

        assertDoesNotThrow(() ->
            BLineCommandsV2.parallel(
                requiringCommand(subsystem),
                requiringCommand(subsystem)
            )
        );
    }

    @Test
    void raceDoesNotInheritChildRequirements() {
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.race(requiringCommand(subsystem));

        assertTrue(command.getRequirements().isEmpty(), "Race group should only see proxy requirements");
    }

    @Test
    void deadlineDoesNotInheritChildRequirements() {
        TestSubsystem deadlineSubsystem = new TestSubsystem();
        TestSubsystem parallelSubsystem = new TestSubsystem();

        Command command = BLineCommandsV2.deadline(
            requiringCommand(deadlineSubsystem),
            requiringCommand(parallelSubsystem)
        );

        assertTrue(command.getRequirements().isEmpty(), "Deadline group should only see proxy requirements");
    }

    @Test
    void eitherDoesNotInheritBranchRequirements() {
        TestSubsystem trueSubsystem = new TestSubsystem();
        TestSubsystem falseSubsystem = new TestSubsystem();

        Command command = BLineCommandsV2.either(
            requiringCommand(trueSubsystem),
            requiringCommand(falseSubsystem),
            () -> true
        );

        assertTrue(command.getRequirements().isEmpty(), "Conditional command should only see proxy requirements");
    }

    @Test
    void selectDoesNotInheritBranchRequirements() {
        TestSubsystem aSubsystem = new TestSubsystem();
        TestSubsystem bSubsystem = new TestSubsystem();
        Map<String, Command> commands = new LinkedHashMap<>();
        commands.put("a", requiringCommand(aSubsystem));
        commands.put("b", requiringCommand(bSubsystem));

        Command command = BLineCommandsV2.select(commands, () -> "a");

        assertTrue(command.getRequirements().isEmpty(), "Select command should only see proxy requirements");
    }

    @Test
    void deferKeepsExplicitRequirementsAndDoesNotInheritSuppliedCommandRequirements() {
        TestSubsystem explicitSubsystem = new TestSubsystem();
        TestSubsystem suppliedSubsystem = new TestSubsystem();

        Command command = BLineCommandsV2.defer(
            () -> requiringCommand(suppliedSubsystem),
            Set.of(explicitSubsystem)
        );

        assertEquals(Set.of(explicitSubsystem), command.getRequirements());
        assertFalse(command.getRequirements().contains(suppliedSubsystem));
    }

    @Test
    void deferredProxyDoesNotDeclareRequirements() {
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.deferredProxy(() -> requiringCommand(subsystem));

        assertTrue(command.getRequirements().isEmpty(), "Deferred proxy should not declare supplied requirements");
    }

    @Test
    void rawSequenceIsInterruptedByEventCommandRequirementConflict() {
        prepareSchedulerRuntime();
        TestSubsystem sharedSubsystem = new TestSubsystem();
        TestSubsystem driveSubsystem = new TestSubsystem();
        AtomicBoolean eventRan = new AtomicBoolean(false);
        AtomicBoolean postRan = new AtomicBoolean(false);
        Command eventCommand = runOnceDisabled(() -> eventRan.set(true), sharedSubsystem);
        Command pathCommand = new EventSchedulingCommand(driveSubsystem, eventCommand, eventRan);

        Command auto = Commands.sequence(
            runOnceDisabled(() -> {}, sharedSubsystem),
            pathCommand,
            runOnceDisabled(() -> postRan.set(true), sharedSubsystem)
        );

        scheduler.schedule(auto);
        runSchedulerUntil(() -> eventRan.get(), 10);
        runSchedulerCycles(5);

        assertTrue(eventRan.get(), "Event command should run after interrupting the raw sequence");
        assertFalse(postRan.get(), "Raw sequence should be canceled before the post-path command");
        assertFalse(scheduler.isScheduled(auto), "Raw sequence should no longer be scheduled");
    }

    @Test
    void bLineSequenceAllowsEventCommandToUseSubsystemFromEarlierAndLaterChildren() {
        prepareSchedulerRuntime();
        TestSubsystem sharedSubsystem = new TestSubsystem();
        TestSubsystem driveSubsystem = new TestSubsystem();
        AtomicBoolean eventRan = new AtomicBoolean(false);
        AtomicBoolean postRan = new AtomicBoolean(false);
        Command eventCommand = runOnceDisabled(() -> eventRan.set(true), sharedSubsystem);
        Command pathCommand = new EventSchedulingCommand(driveSubsystem, eventCommand, eventRan);

        Command auto = BLineCommandsV2.sequence(
            runOnceDisabled(() -> {}, sharedSubsystem),
            pathCommand,
            runOnceDisabled(() -> postRan.set(true), sharedSubsystem)
        );

        scheduler.schedule(auto);
        runSchedulerUntil(() -> postRan.get(), 20);
        runSchedulerCycles(3);

        assertTrue(eventRan.get(), "Event command should run without canceling the BLine sequence");
        assertTrue(postRan.get(), "BLine sequence should continue to the post-path command");
        assertFalse(scheduler.isScheduled(auto), "BLine sequence should finish cleanly");
    }

    @Test
    void eitherRunsSelectedProxiedBranch() {
        prepareSchedulerRuntime();
        AtomicBoolean ran = new AtomicBoolean(false);
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.either(
            runOnceDisabled(() -> ran.set(true), subsystem),
            Commands.none(),
            () -> true
        );

        scheduler.schedule(command);
        runSchedulerUntil(() -> ran.get(), 10);

        assertTrue(ran.get(), "Selected true branch should run through its proxy");
    }

    @Test
    void selectRunsSelectedProxiedBranch() {
        prepareSchedulerRuntime();
        AtomicBoolean ran = new AtomicBoolean(false);
        TestSubsystem subsystem = new TestSubsystem();
        Map<String, Command> commands = new LinkedHashMap<>();
        commands.put("selected", runOnceDisabled(() -> ran.set(true), subsystem));
        commands.put("other", Commands.none());

        Command command = BLineCommandsV2.select(commands, () -> "selected");

        scheduler.schedule(command);
        runSchedulerUntil(() -> ran.get(), 10);

        assertTrue(ran.get(), "Selected map branch should run through its proxy");
    }

    @Test
    void deferRunsSuppliedProxiedCommand() {
        prepareSchedulerRuntime();
        AtomicBoolean ran = new AtomicBoolean(false);
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.defer(
            () -> runOnceDisabled(() -> ran.set(true), subsystem),
            Set.of()
        );

        scheduler.schedule(command.ignoringDisable(true));
        runSchedulerUntil(() -> ran.get(), 10);

        assertTrue(ran.get(), "Deferred supplied command should run through its proxy");
    }

    @Test
    void deferredProxyRunsSuppliedCommand() {
        prepareSchedulerRuntime();
        AtomicBoolean ran = new AtomicBoolean(false);
        TestSubsystem subsystem = new TestSubsystem();

        Command command = BLineCommandsV2.deferredProxy(
            () -> runOnceDisabled(() -> ran.set(true), subsystem)
        );

        scheduler.schedule(command.ignoringDisable(true));
        runSchedulerUntil(() -> ran.get(), 10);

        assertTrue(ran.get(), "Deferred proxy should run supplied command");
    }

    private static Command requiringCommand(Subsystem subsystem) {
        return Commands.run(() -> {}, subsystem).ignoringDisable(true);
    }

    private static Command runOnceDisabled(Runnable action, Subsystem... requirements) {
        return Commands.runOnce(action, requirements).ignoringDisable(true);
    }

    private void prepareSchedulerRuntime() {
        assumeTrue(isWpilibHalRuntimeAvailable(), "WPILib HAL runtime is not available for scheduler tests");
        scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.clearComposedCommands();
        scheduler.enable();
    }

    private static boolean isWpilibHalRuntimeAvailable() {
        try {
            ClassLoader classLoader = BLineCommandsV2Test.class.getClassLoader();
            Class.forName("org.wpilib.hardware.hal.NotifierJNI", false, classLoader);
            Class.forName("org.wpilib.networktables.NetworkTableInstance", false, classLoader);
            Class.forName("com.fasterxml.jackson.databind.ObjectMapper", false, classLoader);
            return isLibraryOnPath("wpiHaljni") &&
                isLibraryOnPath("ntcorejni") &&
                isLibraryOnPath("wpinetjni") &&
                isLibraryOnPath("wpiutil");
        } catch (ClassNotFoundException exception) {
            return false;
        }
    }

    private static boolean isLibraryOnPath(String libraryName) {
        String mappedLibraryName = System.mapLibraryName(libraryName);
        String[] libraryPaths = System.getProperty("java.library.path", "").split(java.io.File.pathSeparator);
        for (String libraryPath : libraryPaths) {
            if (!libraryPath.isBlank() && Files.exists(Path.of(libraryPath, mappedLibraryName))) {
                return true;
            }
        }
        return false;
    }

    private void runSchedulerUntil(BooleanSupplier condition, int maxCycles) {
        for (int i = 0; i < maxCycles && !condition.getAsBoolean(); i++) {
            scheduler.run();
        }
    }

    private void runSchedulerCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            scheduler.run();
        }
    }

    private static final class TestSubsystem implements Subsystem {}

    private static final class EventSchedulingCommand extends Command {
        private final Command eventCommand;
        private final AtomicBoolean eventRan;
        private boolean eventScheduled = false;

        private EventSchedulingCommand(Subsystem driveSubsystem, Command eventCommand, AtomicBoolean eventRan) {
            this.eventCommand = eventCommand;
            this.eventRan = eventRan;
            addRequirements(driveSubsystem);
        }

        @Override
        public void execute() {
            if (!eventScheduled) {
                CommandScheduler.getInstance().schedule(eventCommand);
                eventScheduled = true;
            }
        }

        @Override
        public boolean isFinished() {
            return eventScheduled && eventRan.get();
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}

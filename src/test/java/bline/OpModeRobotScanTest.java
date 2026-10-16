package bline;

import bline.commands.FollowPath;
import bline.commands.FollowPathV2;
import java.io.File;
import java.net.JarURLConnection;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * OpModeRobot's constructor loads every class in every classpath jar under the robot's package
 * with {@code Class.forName(name, false, loader)} and only tolerates ClassNotFoundException and
 * ClassCastException. Each BLine command adapter needs a command framework the robot may not have,
 * so BLine must never ship classes under the template robot package.
 */
class OpModeRobotScanTest {
    private static final String ROBOT_PACKAGE = "frc/robot";
    private static final String COMMANDS_V2 = "org.wpilib.command2.Command";
    private static final String COMMANDS_V3 = "org.wpilib.command3.Command";
    private static final Path BLINE_JAR = Path.of(Objects.requireNonNull(
        System.getProperty("bline.jar"), "Gradle must pass the built BLine jar as bline.jar"));

    @Test
    void noShippedClassLivesInTheRobotPackage() throws Exception {
        try (JarFile jar = new JarFile(BLINE_JAR.toFile())) {
            assertNotNull(jar.getEntry(FollowPath.class.getName().replace('.', '/') + ".class"));
            List<String> robotPackageClasses = jar.stream()
                .map(JarEntry::getName)
                .filter(name -> name.startsWith(ROBOT_PACKAGE) && name.endsWith(".class"))
                .toList();
            assertEquals(List.of(), robotPackageClasses);
        }
    }

    @Test
    void commandsV2OnlyRobotSurvivesTheOpModeScan() throws Exception {
        assertRobotSurvivesTheOpModeScan(COMMANDS_V3, FollowPathV2.class, FollowPath.class);
    }

    @Test
    void commandsV3OnlyRobotSurvivesTheOpModeScan() throws Exception {
        assertRobotSurvivesTheOpModeScan(COMMANDS_V2, FollowPath.class, FollowPathV2.class);
    }

    private static void assertRobotSurvivesTheOpModeScan(
        String missingFramework, Class<?> loadableAdapter, Class<?> unloadableAdapter) throws Exception {
        try (URLClassLoader robot = robotClassLoaderWithout(missingFramework)) {
            assertEquals(List.of(), opModeScanFailures(robot), "OpModeRobot would crash at startup");

            assertThrows(ClassNotFoundException.class, () -> Class.forName(missingFramework, false, robot));
            assertDoesNotThrow(() -> Class.forName(loadableAdapter.getName(), false, robot));
            assertThrows(NoClassDefFoundError.class, () -> Class.forName(unloadableAdapter.getName(), false, robot),
                "The other adapter cannot load here, which is why it must stay out of the scanned package");
        }
    }

    /** The BLine jar plus every test-runtime jar except the command framework the robot did not pick. */
    private static URLClassLoader robotClassLoaderWithout(String missingFramework) throws Exception {
        Path missingJar = Path.of(Class.forName(missingFramework)
            .getProtectionDomain().getCodeSource().getLocation().toURI());
        List<URL> classpath = new ArrayList<>();
        classpath.add(BLINE_JAR.toUri().toURL());
        for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
            Path path = Path.of(entry);
            if (entry.endsWith(".jar") && !path.toAbsolutePath().equals(missingJar.toAbsolutePath())) {
                classpath.add(path.toUri().toURL());
            }
        }
        return new URLClassLoader(classpath.toArray(URL[]::new), ClassLoader.getPlatformClassLoader());
    }

    /** Mirrors OpModeRobot.addAnnotatedOpModeClasses(Package) for a robot in frc.robot. */
    private static List<String> opModeScanFailures(ClassLoader loader) throws Exception {
        Class<?> opMode = Class.forName("org.wpilib.opmode.OpMode", false, loader);
        List<String> failures = new ArrayList<>();
        for (URL root : Collections.list(loader.getResources(ROBOT_PACKAGE))) {
            if (!"jar".equals(root.getProtocol())) {
                continue;
            }
            JarURLConnection connection = (JarURLConnection) root.openConnection();
            connection.setUseCaches(false);
            try (JarFile jar = connection.getJarFile()) {
                for (JarEntry entry : Collections.list(jar.entries())) {
                    String name = entry.getName();
                    if (!name.startsWith(ROBOT_PACKAGE) || !name.endsWith(".class")) {
                        continue;
                    }
                    String className = name.replace('/', '.').substring(0, name.length() - 6);
                    try {
                        Class.forName(className, false, loader).asSubclass(opMode);
                    } catch (ClassNotFoundException | ClassCastException ignored) {
                        // OpModeRobot skips these.
                    } catch (LinkageError error) {
                        failures.add(className + ": " + error);
                    }
                }
            }
        }
        return failures;
    }
}

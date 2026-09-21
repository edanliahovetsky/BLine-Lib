package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertFalse;

import java.nio.file.Path;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.jupiter.api.Test;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

class PublicationMetadataTest {
    @Test
    void publishedPomDoesNotChooseTheRobotCommandFramework() throws Exception {
        var document = DocumentBuilderFactory.newInstance()
            .newDocumentBuilder()
            .parse(Path.of("build/publications/maven/pom-default.xml").toFile());
        NodeList dependencies = document.getElementsByTagName("dependency");
        for (int index = 0; index < dependencies.getLength(); index++) {
            Element dependency = (Element) dependencies.item(index);
            String artifact = text(dependency, "artifactId");
            assertFalse(artifact.equals("commandsv2-java") || artifact.equals("commandsv3-java"),
                "The application must choose its command framework, not BLine's POM");
        }
    }

    private static String text(Element element, String tagName) {
        return element.getElementsByTagName(tagName).item(0).getTextContent();
    }
}

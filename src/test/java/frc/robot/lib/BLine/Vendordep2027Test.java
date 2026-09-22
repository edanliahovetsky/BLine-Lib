package frc.robot.lib.BLine;

import java.nio.file.Files;
import java.nio.file.Path;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

class Vendordep2027Test {
    private static final String BLINE_UUID = "4b7270e9-4e8d-4e7b-8cf0-5805f12c3c7d";
    private static final String CANDIDATE_TAG = "v2027.0.0-beta.1";

    @Test
    void compatibilityVendordepIdentifiesThe2027LineAndExactCandidateTag() throws Exception {
        JSONObject stable = parse("BLine-Lib.json");
        JSONObject compatibility = parse("BLine-Lib-2027.json");

        assertEquals("BLine-Lib-2027.json", compatibility.get("fileName"));
        assertEquals("2027.0.0-beta.1", compatibility.get("version"));
        assertEquals("2027_alpha7", compatibility.get("wpilibYear"));
        assertEquals(BLINE_UUID, compatibility.get("uuid"));
        assertNotEquals(stable.get("jsonUrl"), compatibility.get("jsonUrl"));
        assertEquals(
            "https://raw.githubusercontent.com/edanliahovetsky/BLine-Lib/wpilib-2027/BLine-Lib-2027.json",
            compatibility.get("jsonUrl")
        );

        JSONArray dependencies = (JSONArray) compatibility.get("javaDependencies");
        assertEquals(1, dependencies.size(), "The robot project selects its command framework");
        assertEquals(null, compatibility.get("conflictsWith"));
        JSONObject bline = (JSONObject) dependencies.getFirst();
        assertEquals("com.github.edanliahovetsky", bline.get("groupId"));
        assertEquals("BLine-Lib", bline.get("artifactId"));
        assertEquals(CANDIDATE_TAG, bline.get("version"));

    }

    private static JSONObject parse(String fileName) throws Exception {
        return (JSONObject) new JSONParser().parse(Files.readString(Path.of(fileName)));
    }
}

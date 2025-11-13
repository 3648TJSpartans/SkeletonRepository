package frc.robot.util.pidOptimizer;

import java.io.*;
import java.util.*;

public class RunPython {
    public static void main(String[] args) throws Exception {
        ProcessBuilder pb =
                new ProcessBuilder("python3", "src/main/java/frc/robot/util/pidOptimizer/model.py");
        pb.redirectErrorStream(true);
        Process process = pb.start();

        // Send JSON to Python
        try (BufferedWriter writer =
                new BufferedWriter(new OutputStreamWriter(process.getOutputStream()))) {
            writer.write("{\"data\": [1, 2, 3, 4]}");
            writer.newLine(); // optional, but can help
            writer.flush(); // send immediately
        } // closing the writer signals EOF to Python

        // Read Python output
        BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
        String line;
        StringBuilder output = new StringBuilder();
        while ((line = reader.readLine()) != null) {
            output.append(line);
        }

        int exitCode = process.waitFor();
        System.out.println("Python returned: " + output);
        System.out.println("Exited with code: " + exitCode);
    }
}

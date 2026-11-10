
package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

public class HiveOrientationDetector {

    // ============== HIVE ORIENTATION ============== //

    public enum HiveSide {
        AUDIENCE_UP,
        SCORING_UP,
        UNKNOWN
    }

    // Existing AprilTag processor from TeleOp or autonomous.
    private final AprilTagProcessor processor;

    private final String alliancePrefix;
    private HiveSide currentOrientation = HiveSide.UNKNOWN;

    // TODO: Tune these thresholds after testing the cameras.
    // An upright cluster should have roll near 0 degrees.
    // An inverted cluster should have roll near +/-180 degrees.
    private static final double MAX_UPRIGHT_ROLL = 75.0;


    // ============== INITIALIZATION ============== //

    public HiveOrientationDetector(
            AprilTagProcessor processor,
            boolean redAlliance) {

        this.processor = processor;
        this.alliancePrefix = redAlliance ? "RED" : "BLUE";
    }


    // ============== DETECT HIVE ORIENTATION ============== //

    public void update() {

        HiveSide detectedSide = HiveSide.UNKNOWN;

        for (AprilTagDetection detection : processor.getDetections()) {

            // Only process BIOBUZZ AprilTag clusters.
            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            if (cluster.ftcPose == null) {
                continue;
            }

            String name = cluster.metadata.name.toUpperCase();

            // Ignore the opposing alliance.
            if (!name.startsWith(alliancePrefix + " ")) {
                continue;
            }

            // Check whether this cluster is upright.
            // Visible does not necessarily mean scorable.
            double roll = Math.abs(cluster.ftcPose.roll);

            if (roll >= MAX_UPRIGHT_ROLL) {
                continue;
            }

            HiveSide candidate;

            if (name.contains("AUDIENCE")) {
                candidate = HiveSide.AUDIENCE_UP;

            } else if (name.contains("SCORING")) {
                candidate = HiveSide.SCORING_UP;

            } else {
                continue;
            }

            // Contradictory observations should not produce
            // a confident orientation.
            if (detectedSide != HiveSide.UNKNOWN
                    && detectedSide != candidate) {
                currentOrientation = HiveSide.UNKNOWN;
                return;
            }

            detectedSide = candidate;
        }

        // UNKNOWN when no usable cluster was detected.
        currentOrientation = detectedSide;
    }


    // ============== STATUS METHODS ============== //

    public HiveSide getOrientation() {
        return currentOrientation;
    }

    public boolean isAudienceUp() {
        return currentOrientation == HiveSide.AUDIENCE_UP;
    }

    public boolean isScoringUp() {
        return currentOrientation == HiveSide.SCORING_UP;
    }

    public boolean hasValidObservation() {
        return currentOrientation != HiveSide.UNKNOWN;
    }
}

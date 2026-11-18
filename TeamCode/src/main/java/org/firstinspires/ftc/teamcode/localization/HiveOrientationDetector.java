
package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

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
    private AprilTagClusterDetection stableCluster = null;

    private static final double STABLE_ROLL_A = 0.0;   // Calibrate
    private static final double STABLE_ROLL_B = 0.0;   // Calibrate
    private static final double ROLL_TOLERANCE = 5.0;


    // ============== INITIALIZATION ============== //

    public HiveOrientationDetector(
            AprilTagProcessor processor,
            boolean redAlliance) {

        this.processor = processor;
        this.alliancePrefix = redAlliance ? "RED" : "BLUE";
    }



    private double angleDifference(double a, double b) {
        double difference = Math.toRadians(a - b);

        return Math.toDegrees(Math.atan2(
                Math.sin(difference),
                Math.cos(difference)
        ));
    }

    public boolean isStable(double measuredRoll) {
        return Math.abs(angleDifference(
                measuredRoll, STABLE_ROLL_A
        )) <= ROLL_TOLERANCE

                || Math.abs(angleDifference(
                measuredRoll, STABLE_ROLL_B
        )) <= ROLL_TOLERANCE;
    }

    // ============== DETECT HIVE ORIENTATION ============== //

    public void update(List<AprilTagDetection> detections) {

        HiveSide detectedSide = HiveSide.UNKNOWN;

        // Reset the stable cluster every cycle to avoid stale data
        stableCluster = null;

        for (AprilTagDetection detection : detections) {

            // Only process BIOBUZZ AprilTag clusters.
            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

            if (cluster.ftcPose == null) {
                continue;
            }

            if (cluster.metadata == null || cluster.metadata.name == null) {
                continue;
            }

            String name = cluster.metadata.name.toUpperCase();

            // Ignore the opposing alliance.
            if (!name.startsWith(alliancePrefix + " ")) {
                continue;
            }

            // Check whether this cluster is upright.
            // Visible does not necessarily mean scorable.
            double roll = cluster.ftcPose.roll;

            if (!(isStable(roll))) {
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
                stableCluster = null;
                return;
            }

            detectedSide = candidate;
            stableCluster = cluster;
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
    public AprilTagClusterDetection getStableCluster() { return stableCluster; }
}

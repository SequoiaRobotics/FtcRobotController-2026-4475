
package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.math.Pose;

public class CameraOdometryCorrector {

    // =====================================================
    // CONFIGURATION
    // =====================================================

    // TODO: Eventually enable corrections only after testing.
    // For now, this class only compares measurements.

    // =====================================================
    // CAMERA MEASUREMENT
    // =====================================================

    private Pose cameraPose = null;
    private boolean hasNewMeasurement = false;

    // Difference between odometry and camera estimates.
    private double positionError = 0;
    private double headingError = 0;


    // =====================================================
    // RECEIVE CAMERA POSE
    // =====================================================

    public void submitCameraPose(Pose measuredPose) {

        // TODO: Later verify:
        // - AprilTag identity and field position
        // - Camera calibration and mounting offsets
        // - Measurement quality and timestamp
        // - Coordinate system compatibility with Pedro

        if (measuredPose == null) {
            return;
        }

        cameraPose = measuredPose;
        hasNewMeasurement = true;
    }


    // =====================================================
    // COMPARE CAMERA AND ODOMETRY
    // =====================================================

    public void update(Pose odometryPose) {

        // No new usable AprilTag measurement?
        // Continue using Pinpoint without doing anything.
        if (!hasNewMeasurement || odometryPose == null) {
            return;
        }

        hasNewMeasurement = false;

        double dx = cameraPose.x() - odometryPose.x();
        double dy = cameraPose.y() - odometryPose.y();

        positionError = Math.hypot(dx, dy);

        // Both headings are in radians.
        double difference =
                cameraPose.heading() - odometryPose.heading();

        headingError = Math.atan2(
                Math.sin(difference),
                Math.cos(difference)
        );

        // TODO: Decide whether the measurement is reliable.
        // TODO: Reject outliers and stale measurements.
        // TODO: Account for camera measurement latency.
        // TODO: Apply a carefully tested pose correction.
        //
        // Do NOT call follower.setPose() automatically yet.
    }


    // =====================================================
    // TELEMETRY
    // =====================================================

    public double getPositionError() {
        return positionError;
    }

    public double getHeadingErrorDegrees() {
        return Math.toDegrees(headingError);
    }
}

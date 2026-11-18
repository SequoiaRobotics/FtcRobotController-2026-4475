
package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class CameraOdometryCorrector {

    // Pose factory
    private final PoseFactory p = PoseFactory.degrees();
    private Pose cameraPose = null;
    private boolean hasNewMeasurement = false;

    // Difference between odometry and camera estimates.
    private double positionError = 0;
    private double headingError = 0;

    // Receive a camera pose
    public void submitCameraPose(Pose measuredPose) {
        if (measuredPose == null) {
            return;
        }

        cameraPose = measuredPose;
        hasNewMeasurement = true;
    }

    public Pose toPedroPose(
            double xFTC, double yFTC,
            double headingFTC) {

        double xPedro = yFTC + 72.0;
        double yPedro = 72.0 - xFTC;

        double headingPedro = headingFTC - 90.0;

        return p.of(xPedro, yPedro, headingPedro);
    }


    // Compare the camera pose to odometry, and determine the difference
    public void update(Pose odometryPose) {
        // If there's no new measurement, stick to pinpoint odometry
        if (!hasNewMeasurement) {
            return;
        }

        double dx = cameraPose.x() - odometryPose.x();
        double dy = cameraPose.y() - odometryPose.y();

        // Calculate the position error with the Pythagorean theorem
        positionError = Math.hypot(dx, dy);

        // Radians
        double dh = cameraPose.heading() - odometryPose.heading();

        // Construct this angle on the unit circle, and then use the vector to
        // the origin to determine an angle between -180 and 180.
        headingError = Math.atan2(Math.sin(dh), Math.cos(dh));
    }


    // =====================================================
    // TELEMETRY
    // =====================================================

    public double getPositionError() {
        return positionError;
    }

    public double getHeadingErrorDegrees() {
        // Convert the default radians into degrees
        return Math.toDegrees(headingError);
    }
}

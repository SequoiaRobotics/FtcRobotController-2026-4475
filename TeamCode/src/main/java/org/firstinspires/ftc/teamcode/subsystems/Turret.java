
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;

public class Turret {

    // Target coordinates in Pedro's field coordinate system (inches).
    private double targetX;
    private double targetY;

    // Requested turret angle relative to robot forward (degrees).
    private double targetAngle = 0;

    // True when field-target tracking is active.
    private boolean tracking = false;

    // TODO: Measure turret pivot position relative to robot center.
    // Positive forward = toward robot front.
    // Positive left = toward robot left.
    private static final double TURRET_FORWARD_OFFSET = 0.0;
    private static final double TURRET_LEFT_OFFSET = 0.0;

    public Turret() {
        // TODO: Initialize turret motor and angle sensor
        // when the actual hardware is installed.
    }

    // Begin tracking a fixed target in field coordinates.
    public void setFieldTarget(double x, double y) {
        targetX = x;
        targetY = y;
        tracking = true;
    }

    // Manually request a turret angle.
    public void setTargetAngle(double degrees) {
        tracking = false;
        targetAngle = normalizeAngle(degrees);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isTracking() {
        return tracking;
    }

    // Call every OpMode loop with the latest Pedro pose.
    public void update(Pose robotPose) {

        if (tracking) {
            double heading = robotPose.heading(); // Radians

            // Calculate turret pivot position in FIELD coordinates.
            double turretX = robotPose.x()
                    + Math.cos(heading) * TURRET_FORWARD_OFFSET
                    - Math.sin(heading) * TURRET_LEFT_OFFSET;

            double turretY = robotPose.y()
                    + Math.sin(heading) * TURRET_FORWARD_OFFSET
                    + Math.cos(heading) * TURRET_LEFT_OFFSET;

            // Vector from turret pivot to target.
            double dx = targetX - turretX;
            double dy = targetY - turretY;

            // Do not change aim if target coincides with pivot.
            if (Math.hypot(dx, dy) > 0.001) {

                // Absolute direction toward target.
                double fieldAngle = Math.atan2(dy, dx);

                // Convert to robot-relative turret angle.
                double relativeAngle = fieldAngle - heading;

                targetAngle = normalizeAngle(
                        Math.toDegrees(relativeAngle)
                );
            }
        }

        // TODO: Read actual turret encoder angle.
        // TODO: Check mechanical rotation limits.
        // TODO: Command motor toward targetAngle.
        // TODO: Use feedback to reduce tracking error.
    }

    // Normalize an angle into [-180, 180).
    private double normalizeAngle(double degrees) {
        return ((degrees + 180) % 360 + 360) % 360 - 180;
    }

    public void stop() {
        tracking = false;
        // TODO: Stop turret motor safely.
    }
}

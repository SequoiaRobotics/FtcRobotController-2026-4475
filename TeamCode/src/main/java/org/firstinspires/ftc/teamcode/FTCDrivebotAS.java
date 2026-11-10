package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "FTCDrivebotAS")
public class FTCDrivebotAS extends LinearOpMode {

    // Set false if only Webcam 1 is connected.
    private static final boolean USE_SECOND_CAMERA = true;

    // Groups each camera's vision processor with its portal.
    private static final class CameraSession {
        final String name;
        final AprilTagProcessor processor;
        final VisionPortal portal;

        CameraSession(String name, AprilTagProcessor processor, VisionPortal portal) {
            this.name = name;
            this.processor = processor;
            this.portal = portal;
        }
    }

    private CameraSession initializeCamera(
            String hardwareName,
            Size resolution,
            double right,
            double forward,
            double up,
            double yaw,
            double pitch,
            double roll) {

        // TODO: Replace the sample offsets/angles with measured camera mounting values.
        Position cameraPosition = new Position(
                DistanceUnit.INCH, right, forward, up, 0
        );
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
                AngleUnit.DEGREES, yaw, pitch, roll, 0
        );

        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, hardwareName))
                .setCameraResolution(resolution)
                .addProcessor(processor)
                // Disable RC live preview, especially with two simultaneous cameras.
                .enableLiveView(false)
                .build();

        return new CameraSession(hardwareName, processor, portal);
    }

    private void addCameraTelemetry(CameraSession camera) {
        if (camera == null) {
            return;
        }

        String prefix = camera.name + " / ";
        telemetry.addData(prefix + "state", camera.portal.getCameraState());
        telemetry.addData(prefix + "FPS", "%.1f", camera.portal.getFps());

        List<AprilTagDetection> detections = camera.processor.getDetections();
        telemetry.addData(prefix + "detections", detections.size());

        int index = 0;
        for (AprilTagDetection detection : detections) {
            index++;

            if (!(detection instanceof AprilTagClusterDetection)) {
                telemetry.addData(prefix + "detection " + index,
                        "Standalone tag (not displayed by cluster telemetry)");
                continue;
            }

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            String label = cluster.metadata != null
                    ? cluster.metadata.name
                    : "Unnamed cluster";
            String target = prefix + index + " " + label + " / ";

            telemetry.addData(target + "tags visible", "%d%%", cluster.percentClusterFound);

            if (cluster.ftcPose == null) {
                telemetry.addData(target + "pose", "Unavailable");
                continue;
            }

            // These are camera-relative detections, NOT Pedro field coordinates.
            telemetry.addData(target + "camera XYZ (in)", "%.1f, %.1f, %.1f",
                    cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z);
            telemetry.addData(target + "pitch/roll/yaw (deg)", "%.1f, %.1f, %.1f",
                    cluster.ftcPose.pitch, cluster.ftcPose.roll, cluster.ftcPose.yaw);
        }
    }

    private void closeCamera(CameraSession camera) {
        if (camera != null && camera.portal != null) {
            camera.portal.close();
        }
    }

    @Override
    public void runOpMode() {
        // Original Android Studio drivetrain setup, unchanged.
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CameraSession camera1 = null;
        CameraSession camera2 = null;

        try {
            // TODO: Camera mounting values are placeholders:
            // right, forward, up (inches), yaw, pitch, roll (degrees).
            camera1 = initializeCamera(
                    "Webcam 1", new Size(640, 480),
                    9, 9, 5,
                    0, -90, 0
            );

            if (USE_SECOND_CAMERA) {
                camera2 = initializeCamera(
                        "Webcam 2", new Size(640, 480),
                        -9, 9, 5,
                        0, -90, 0
                );
            }

            // Show vision diagnostics before pressing Start.
            while (opModeInInit()) {
                addCameraTelemetry(camera1);
                addCameraTelemetry(camera2);
                telemetry.update();
                sleep(50);
            }

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                // Original Android Studio drive controls, unchanged.
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                // Soften joystick inputs while preserving their signs.
                forward = Math.copySign(forward * forward, forward);
                strafe = Math.copySign(strafe * strafe, strafe);
                turn = Math.copySign(turn * turn, turn);

                double frontLeftPower = forward + strafe + turn;
                double backLeftPower = forward - strafe + turn;
                double frontRightPower = forward - strafe - turn;
                double backRightPower = forward + strafe - turn;

                // Normalize together to preserve the requested direction.
                double max = Math.max(
                        1.0,
                        Math.max(
                                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
                        )
                );

                frontLeftMotor.setPower(frontLeftPower / max);
                backLeftMotor.setPower(backLeftPower / max);
                frontRightMotor.setPower(frontRightPower / max);
                backRightMotor.setPower(backRightPower / max);

                // Read camera detections for telemetry only.
                // Nothing here changes drivetrain control or Pinpoint/Pedro localization.
                addCameraTelemetry(camera1);
                addCameraTelemetry(camera2);
                telemetry.update();

                sleep(20);
            }
        } finally {
            try {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            } finally {
                try {
                    closeCamera(camera2);
                } finally {
                    closeCamera(camera1);
                }
            }
        }
    }
}

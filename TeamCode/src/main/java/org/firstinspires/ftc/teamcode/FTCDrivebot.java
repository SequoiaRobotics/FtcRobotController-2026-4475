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

@TeleOp
public class FTCDrivebot extends LinearOpMode {

    private static final class CameraSession {
        final String name;
        final AprilTagProcessor processor;
        final VisionPortal portal;

        CameraSession(
                String name,
                AprilTagProcessor processor,
                VisionPortal portal) {
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

        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                right, forward, up,
                0
        );

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                yaw, pitch, roll,
                0
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
                // SDK 12 API. Disable RC previews to avoid sharing
                // the default preview container between two portals.
                .enableLiveView(false)
                .build();

        return new CameraSession(hardwareName, processor, portal);
    }

   private void addCameraTelemetry(CameraSession camera) {
        if (camera == null) {
            return;
        }

        String prefix = camera.name + " / ";

        //keeping status and UI compact
        telemetry.addData(
                prefix + "Status",
                "State: %s | FPS: %.1f",
                camera.portal.getCameraState(),
                camera.portal.getFps()
        );

        List<AprilTagDetection> detections =
                camera.processor.getDetections();

        telemetry.addData(prefix + "Detections Found", detections.size());

        int index = 0;

        for (AprilTagDetection detection : detections) {
            index++;

            if (!(detection instanceof AprilTagClusterDetection)) {
                //using single line instead of skipping a log line entirely
                telemetry.addData(prefix + "Tag #" + index, "Standalone (Skipped)");
                continue;
            }

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            String targetName = (cluster.metadata != null) ? cluster.metadata.name : "ID " + cluster.id;

            //combibging into clean & singleline summaries per tag
            if (cluster.ftcPose != null) {
                telemetry.addData(
                        prefix + "Tag " + index + " [" + targetName + "]",
                        "Vis: %d%% | XYZ(in): %.1f, %.1f, %.1f | R/P/Y: %.0f, %.0f, %.0f",
                        (int) cluster.percentClusterFound,
                        cluster.ftcPose.x,
                        cluster.ftcPose.y,
                        cluster.ftcPose.z,
                        cluster.ftcPose.roll,
                        cluster.ftcPose.pitch,
                        cluster.ftcPose.yaw
                );
            } else {
                telemetry.addData(
                        prefix + "Tag " + index + " [" + targetName + "]",
                        "Vis: %d%% | Pose Unavailable",
                        (int) cluster.percentClusterFound
                );
            }
        }
    }
    private void closeCamera(CameraSession camera) {
        if (camera != null && camera.portal != null) {
            camera.portal.close();
        }
    }

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor =
                hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeftMotor =
                hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRightMotor =
                hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRightMotor =
                hardwareMap.get(DcMotor.class, "backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /* frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
        
        // using RUN_WITHOUT_ENCODER to maximize raw speed as said online

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CameraSession webcam = null;

        try {
            // PLACEHOLDERS: replace with measured mounting values.
            // Arguments after resolution:
            // right, forward, up (inches), yaw, pitch, roll (degrees).
            webcam = initializeCamera(
                    "Webcam 1",
                    new Size(1280, 720),
                    0,  //right
                    0,  //foreward
                    5,  //up
                    0,  //yaw
                    45, //pitch
                    0   //roll
            );

            // Show camera diagnostics during INIT.
            while (opModeInInit()) {
                addCameraTelemetry(webcam);
                telemetry.update();
                sleep(50);
            }

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                // Joystick softening, preserving direction.
                forward = Math.copySign(forward * forward, forward);
                strafe = Math.copySign(strafe * strafe, strafe);
                turn = Math.copySign(turn * turn, turn);

                double frontLeftPower = forward + strafe + turn;
                double backLeftPower = forward - strafe + turn;
                double frontRightPower = forward - strafe - turn;
                double backRightPower = forward + strafe - turn;

                // Scale all powers proportionally to keep values in -1 --> 1 range
                double max = Math.max(
                        1.0,
                        Math.max(
                                Math.max(
                                        Math.abs(frontLeftPower),
                                        Math.abs(backLeftPower)
                                ),
                                Math.max(
                                        Math.abs(frontRightPower),
                                        Math.abs(backRightPower)
                                )
                        )
                );

                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                addCameraTelemetry(webcam);
                telemetry.update();

                sleep(20);
            }
        } finally {
            // Attempt camera cleanup even if motor cleanup throws.
            try {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            } finally {
                    closeCamera(webcam);
            
                }
            }
        }
    }
}

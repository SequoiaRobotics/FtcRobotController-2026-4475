package org.firstinspires.ftc.teamcode.movement;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.localization.CameraSession;
import org.firstinspires.ftc.teamcode.subsystems.TurretStationary;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.List;

@TeleOp(name = "FTCDrivebotAS")
public class FTCDrivebotAS extends LinearOpMode {

    // Set false if only Webcam 1 is connected.
    private static final boolean USE_SECOND_CAMERA = false;
    private static final boolean USE_VISION = false;
    Intake intake;
    TurretStationary turretStationary;
    int intakeDirection = 0; // 0 = OFF, 1 = ON, -1 = REVERSE
    private int indexerDirection = 0; // 0 OFF, 1 FORWARD, -1 REVERSE
    boolean turretOn = false;

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

        //orienting gyroscope in control hub
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )
        );

        imu.initialize(parameters);
        
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

        //Makes robot stop after let go of joystick
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CameraSession camera1 = null;
        CameraSession camera2 = null;

        intake = new Intake(hardwareMap);
        turretStationary = new TurretStationary(hardwareMap);

        try {
            // TODO: Camera mounting values are placeholders:
            // right, forward, up (inches), yaw, pitch, roll (degrees).
            if (USE_VISION) {
                camera1 = CameraSession.initializeCamera(
                        hardwareMap, "Webcam 1", new Size(640, 480),
                        9, 9, 5,
                        0, -90, 0
                );


                if (USE_SECOND_CAMERA) {
                    camera2 = CameraSession.initializeCamera(
                            hardwareMap, "Webcam 2", new Size(640, 480),
                            -9, 9, 5,
                            0, -90, 0
                    );
                }
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

                //Reset robots forward direction to 0 degrees if driver presses options
            if (gamepad1.options) {
                imu.resetYaw();
            }
                // Turret update
                turretStationary.update();

                // Original Android Studio drive controls, unchanged. (DRIVER GAMEPAD)
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;


                // ============== INTAKE CONTROLS ============== //

                // Triangle: Forward intake (SHOOTER GAMEPAD)
                if (gamepad2.triangleWasPressed()) {
                    intakeDirection = (intakeDirection == 1) ? 0 : 1;
                }

                // X (Cross): Reverse intake (SHOOTER GAMEPAD)
                if (gamepad2.aWasPressed()) {
                    intakeDirection = (intakeDirection == -1) ? 0 : -1;
                }

                if (intakeDirection == 1) {
                    intake.start();
                } else if (intakeDirection == -1) {
                    intake.reverse();
                } else {
                    intake.stop();
                }


                // ============== INDEXER CONTROLS ============== //

                // D-pad Up: Forward indexer (SHOOTER GAMEPAD)

                if (gamepad2.dpadUpWasPressed()) {
                    indexerDirection = (indexerDirection == 1) ? 0 : 1;
                }

                // D-pad Down: Reverse indexer (SHOOTER GAMEPAD)
                if (gamepad2.dpadDownWasPressed()) {
                    indexerDirection = (indexerDirection == -1) ? 0 : -1;
                }

                if (indexerDirection == 1) {
                    turretStationary.indexerOn();
                } else if (indexerDirection == -1) {
                    turretStationary.indexerReverse();
                } else {
                    turretStationary.indexerOff();
                }




                // ============== FLYWHEEL CONTROLS ============== //

                // Square: Toggle flywheel (SHOOTER GAMEPAD)
                if (gamepad2.squareWasPressed()) {
                    turretOn = !turretOn;
                }

                if (turretOn) {
                    turretStationary.startFlyWheel();
                } else {
                    turretStationary.stopFlyWheel();
                }

                turretStationary.update();





                // Flip the boolean value each time square is newly pressed
                if (gamepad1.squareWasPressed()) {
                    turretOn = !turretOn;
                }

                if (turretOn) {
                    turretStationary.startFlyWheel();
                } else {
                    turretStationary.stopFlyWheel();
                }

                
                //getting robotics current heading
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                //rotate jostick inputs using heading
                double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
                double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);


                //new soften joystick inputs with rotY and rotX instead of forward and strafe
                rotY = Math.copySign(rotY * rotY, rotY);
                rotX = Math.copySign(rotX * rotX, rotX);
                turn = Math.copySign(turn * turn, turn);
                /*
                // Soften joystick inputs while preserving their signs.
                forward = Math.copySign(forward * forward, forward);
                strafe = Math.copySign(strafe * strafe, strafe);
                turn = Math.copySign(turn * turn, turn);
                */

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

                if (intake != null) {
                    intake.stop();
                }

                if (turretStationary != null) {
                    turretStationary.stop();
                }


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




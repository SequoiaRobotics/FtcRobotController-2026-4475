package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.gorillacoder.AbstractBasicDriveBot.DriveMode.DRIVE_POV;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.gorillacoder.AbstractBasicTeleOpDriveBot;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
@TeleOp
public class GorillaDriveBot extends AbstractBasicTeleOpDriveBot {

    protected double lastDriveModeSwitchTime = getRuntime();

    @SuppressWarnings("StatementWithEmptyBody")
    protected void readGamePad() {
        if (getRuntime() < lastDriveModeSwitchTime + .2) {
            // debounce, don't accept too rapid mode switches. We just run too fast.
            // The user is holding the button down, and we detect it twice (or more).
        } else if (gamepad1.right_stick_button) {
            // change driveMode
            driveMode = nextDriveModeFor.get(driveMode);
            lastDriveModeSwitchTime = getRuntime();
        }
        if (null == driveMode) {
            driveMode = DRIVE_POV;
        }
        switch (driveMode) {
        case DRIVE_POV:
            drivePovTask.speed    = gamepad1.right_stick_y;
            drivePovTask.turnRate = gamepad1.right_stick_x;
            break;
        case DRIVE_TANK:
            driveTankTask.leftPower  = gamepad1.left_stick_y;
            driveTankTask.rightPower = gamepad1.right_stick_y;
            break;
        case DRIVE_OMNI:
            driveOmniTask.axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            driveOmniTask.lateral = gamepad1.left_stick_x;
            driveOmniTask.yaw     = gamepad1.right_stick_x;
            break;
        }

        if (gamepad1.left_bumper) {
            telemetryMode = TelemetryMode.TELEMETRY_LOCATIONS;
        } else if (gamepad1.right_bumper) {
            telemetryMode = TelemetryMode.TELEMETRY_APRIL_TAG_NAMES;
        }
    }

    protected enum TelemetryMode {
        TELEMETRY_LOCATIONS,
        TELEMETRY_APRIL_TAG_NAMES
    }
    protected TelemetryMode telemetryMode = TelemetryMode.TELEMETRY_LOCATIONS;
    protected Map<TelemetryMode, TelemetryMode> nextTelemetryModeFor = new HashMap<>();
    {
        nextTelemetryModeFor.put(TelemetryMode.TELEMETRY_LOCATIONS,       TelemetryMode.TELEMETRY_APRIL_TAG_NAMES);
        nextTelemetryModeFor.put(TelemetryMode.TELEMETRY_APRIL_TAG_NAMES, TelemetryMode.TELEMETRY_LOCATIONS);
    }

    @Override
    protected void updateTelemetry() {
        telemetry.log().clear();

        telemetry.addData("Status", "%s alliance running %s", alliance, runtime);
        telemetry.addData("Camera", "%s %s", visionTask.portalLeft.getCameraState() , visionTask.portalRight.getCameraState());

        switch (driveMode) {
            case DRIVE_POV:
                telemetry.addData("POV", "speed:%.2f  turn:%.2f", drivePovTask.speed, drivePovTask.turnRate);
                telemetry.addData("   ", " left:%.2f right:%.2f", drivePovTask.leftPower, drivePovTask.rightPower);
                break;
            case DRIVE_TANK:
                telemetry.addData("TANK", " left:%.2f right:%.2f", driveTankTask.leftPower, driveTankTask.rightPower);
                break;
            case DRIVE_OMNI:
                telemetry.addData("OMNI", "axial:%.2f  lat:%.2f yaw:%.2f", driveOmniTask.axial, driveOmniTask.lateral, driveOmniTask.yaw);
                telemetry.addData("    ", "motors:%.2f %.2f %.2f %.2f",
                    driveOmniTask.driveLeftFront.getPower(),
                    driveOmniTask.driveRightFront.getPower(),
                    driveOmniTask.driveLeftFront.getPower(),
                    driveOmniTask.driveRightFront.getPower()
                );
                break;
            default:
                telemetry.addData("WEIRD MOTOR MODE", "");
        }

        switch (telemetryMode) {
            case TELEMETRY_LOCATIONS:
                updateTelemetryLocations();
                break;
            case TELEMETRY_APRIL_TAG_NAMES:
                updateTelemetryAprilTags();
                break;
        }
    }
    @Override
    protected GorillaDriveBot configureAprilTagProcessors() {
        visionTask.atpBuilderLeft. setCameraPose(cameraPositionLeft,  cameraOrientationLeft);
        visionTask.atpBuilderRight.setCameraPose(cameraPositionRight, cameraOrientationRight);

        return this;
    }

    static {
        cameraPositionLeft     = new Position(INCH, -6.5, 7., 10., 0);
        cameraOrientationLeft  = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);

        cameraPositionRight    = new Position(INCH,  6.5, 7., 10., 0);
        cameraOrientationRight = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);
    }

} // class GorillaDriveBot2

// 2025 code
//@Override
//protected void updateTelemetry() {
//    telemetry.addData("Status", "Version 1. %s alliance running %s", alliance, runtime);
//    telemetry.addData("Camera", "%s %s", visionTask.portalLeft.getCameraState() , visionTask.portalRight.getCameraState());
//    telemetry.addData("Motors", "speed:%.2f  turn:%.2f", drivePovTask.speed, drivePovTask.turnRate);
//    telemetry.addData("Motors", " left:%.2f right:%.2f", drivePovTask.leftPower, drivePovTask.rightPower);
//
//        // Red Tower seen from left camera
//        AprilTagDetection redLeftDetection   = visionTask.targetDetectionsLeft.get("24");
//        String            towerRedLeft       = "b:? r:? y:?";
//        String            botRedLeft         = "x:? y:? y:?";
//        // Blue Tower seen from left camera
//        AprilTagDetection blueLeftDetection  = visionTask.targetDetectionsLeft.get("20");
//        String            towerBlueLeft      = "b:? r:? y:?";
//        String            botBlueLeft        = "x:? y:? y:?";
//        // Red Tower seen from right camera
//        AprilTagDetection redRightDetection  = visionTask.targetDetectionsRight.get("24");
//        String            towerRedRight      = "b:? r:? y:?";
//        String            botRedRight        = "x:? y:? y:?";
//        // Blue Tower seen from right camera
//        AprilTagDetection blueRightDetection = visionTask.targetDetectionsRight.get("20");
//        String            towerBlueRight     = "b:? r:? y:?";
//        String            botBlueRight       = "x:? y:? y:?";
//
//        if (null != redLeftDetection) {
//            towerRedLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
//                redLeftDetection.ftcPose.bearing, redLeftDetection.ftcPose.range, redLeftDetection.ftcPose.yaw);
//            botRedLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
//                redLeftDetection.robotPose.getPosition().x, redLeftDetection.robotPose.getPosition().y, redLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
//        }
//        if (null != blueLeftDetection) {
//            towerBlueLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
//                blueLeftDetection.ftcPose.bearing, blueLeftDetection.ftcPose.range, blueLeftDetection.ftcPose.yaw);
//            botBlueLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
//                blueLeftDetection.robotPose.getPosition().x, blueLeftDetection.robotPose.getPosition().y, blueLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
//        }
//        if (null != redRightDetection) {
//            towerRedRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
//                redRightDetection.ftcPose.bearing, redRightDetection.ftcPose.range, redRightDetection.ftcPose.yaw);
//            botRedRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
//                redRightDetection.robotPose.getPosition().x, redRightDetection.robotPose.getPosition().y, redRightDetection.robotPose.getOrientation().getYaw(DEGREES));
//        }
//        if (null != blueRightDetection) {
//            towerBlueRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
//                blueRightDetection.ftcPose.bearing, blueRightDetection.ftcPose.range, blueRightDetection.ftcPose.yaw);
//            botBlueRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
//                blueRightDetection.robotPose.getPosition().x, blueRightDetection.robotPose.getPosition().y, blueRightDetection.robotPose.getOrientation().getYaw(DEGREES));
//        }
//
//        // Update blob data
//
//        List<Blob> greenBlobs     = blobLocatorGreenRight.getBlobs();
//        List<Blob> purpleBlobs    = blobLocatorPurpleLeft.getBlobs();
//        String     greenBlobInfo  = " 0 #1: (  ?,   ?)\na:    d:     r:     l:      c:";
//        String     purpleBlobInfo = " 0 #1: (  ?,   ?)\na:    d:     r:     l:      c:";
//        if (!greenBlobs.isEmpty()) {
//            Blob blob = greenBlobs.get(0);
//            greenBlobInfo = String.format(
//                Locale.US, "%2d #1: (%3.0f, %3.0f)\na:%3d d:%.2f r:%.2f l:%6.2f c:%.2f",
//                greenBlobs.size(), blob.getCircle().getX(), blob.getCircle().getY(),
//                blob.getContourArea(), blob.getDensity(), blob.getAspectRatio(), blob.getArcLength(), blob.getCircularity()
//            );
//        }
//        if (!purpleBlobs.isEmpty()) {
//            Blob blob = purpleBlobs.get(0);
//            purpleBlobInfo = String.format(
//                Locale.US, "%2d #1: (%3.0f, %3.0f)\na:%3d d:%.2f r:%.2f l:%6.2f c:%.2f",
//                purpleBlobs.size(), blob.getCircle().getX(), blob.getCircle().getY(),
//                blob.getContourArea(), blob.getDensity(), blob.getAspectRatio(), blob.getArcLength(), blob.getCircularity()
//            );
//        }
//
//        telemetry.addData("Bot RL", "%s", botRedLeft);
//        telemetry.addData("    RR", "%s", botRedRight);
//        telemetry.addData("    BL", "%s", botBlueLeft);
//        telemetry.addData("    BR", "%s", botBlueRight);
//        telemetry.addData("Twr RL", "%s", towerRedLeft);
//        telemetry.addData("    RR", "%s", towerRedRight);
//        telemetry.addData("    BL", "%s", towerBlueLeft);
//        telemetry.addData("    BR", "%s", towerBlueRight);
//        telemetry.addData("Green ", "%s", greenBlobInfo);
//        telemetry.addData("Purple", "%s", purpleBlobInfo);
//}


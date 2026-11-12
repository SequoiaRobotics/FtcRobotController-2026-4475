package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.gorillacoder.AbstractBasicDriveBot.DriveMode.DRIVE_POV;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.gorillacoder.AbstractBasicDriveBot;
import org.firstinspires.ftc.gorillacoder.AbstractBasicTeleOpDriveBot;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.AbstractMap;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.stream.StreamSupport;

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
            driveOmniTask.axial   = gamepad1.left_stick_y;
            driveOmniTask.lateral = -gamepad1.left_stick_x;
            driveOmniTask.yaw     = -gamepad1.right_stick_x;
            break;
        }

        if (gamepad1.left_bumper) {
            telemetryMode = TelemetryMode.TELEMETRY_GAME_LOCATIONS;
        } else if (gamepad1.right_bumper) {
            telemetryMode = TelemetryMode.TELEMETRY_APRIL_TAG_NAMES;
        }
    }

    protected enum TelemetryMode {
        TELEMETRY_GAME_LOCATIONS,
        TELEMETRY_TEST_LOCATIONS,
        TELEMETRY_APRIL_TAG_NAMES
    }
    protected TelemetryMode telemetryMode = TelemetryMode.TELEMETRY_GAME_LOCATIONS;
    protected Map<TelemetryMode, TelemetryMode> nextTelemetryModeFor = new HashMap<>();
    {
        nextTelemetryModeFor.put(TelemetryMode.TELEMETRY_GAME_LOCATIONS,  TelemetryMode.TELEMETRY_TEST_LOCATIONS);
        nextTelemetryModeFor.put(TelemetryMode.TELEMETRY_TEST_LOCATIONS,  TelemetryMode.TELEMETRY_APRIL_TAG_NAMES);
        nextTelemetryModeFor.put(TelemetryMode.TELEMETRY_APRIL_TAG_NAMES, TelemetryMode.TELEMETRY_GAME_LOCATIONS);
    }

    @Override
    protected void updateTelemetry() {
        telemetry.log().clear();

        String last = "duration:??? distance:???";
        String location = "x:??? y:??? h:???";

        if ( null != this.latestDetection && null != this.latestDetection.robotPose ) {

            last = String.format(
                Locale.US,
                "duration:%d distance:???", // TO DO: distance from encoders since last detection
                System.nanoTime() - this.latestDetectionAcquisitionTime
            );
            location = String.format(
                Locale.US,
                "x:%0.2d y:%0.2d h:%0.2d",
                this.latestDetection.robotPose.getPosition().x,
                this.latestDetection.robotPose.getPosition().y,
                this.latestDetection.robotPose.getOrientation().getYaw()
            );
        }
        telemetry.addData("Status", "%s alliance running %s", alliance, runtime);
        telemetry.addData( "Last", "%s",     last);
        telemetry.addData("Location", "%s",  location );
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

        updateTelemetryGameLocations();

//        switch (telemetryMode) {
//            case TELEMETRY_GAME_LOCATIONS:
//                updateTelemetryGameLocations();
//                break;
//            case TELEMETRY_TEST_LOCATIONS:
//                updateTelemetryTestLocations();
//                break;
//            case TELEMETRY_APRIL_TAG_NAMES:
//                updateTelemetryAprilTagNames();
//                break;
//        }

        if (this.running) return;

        telemetry.addData( "config", getActiveConfigName() );
        hardwareMap.getAllNames(HardwareDevice.class).stream()
            .map(name -> new AbstractMap.SimpleEntry<>(name, hardwareMap.get(name)))
            .sorted(Comparator.comparing(
                mapping -> mapping.getValue().getConnectionInfo(),
                Comparator.nullsLast(String::compareToIgnoreCase)
            ))
            .forEach(mapping -> {
                telemetry.addData(mapping.getKey(), mapping.getValue().getConnectionInfo());
            });
    }

    // TO DO: Move run state to AbstractOpMode.
    // Investigate using OpModeManagerNotifier
    private boolean running = false;
    public void loop() {
        super.loop();
        running = true;
    }

    private String getActiveConfigName() {
        RobotConfigFileManager configFileManager = new RobotConfigFileManager();
        RobotConfigFile activeConfig = configFileManager.getActiveConfig();
        return activeConfig.getName();
    }

    @Override
    protected GorillaDriveBot configureAprilTagProcessors() {
        visionTask.atpBuilderLeft. setCameraPose(cameraPositionLeft,  cameraOrientationLeft);
        visionTask.atpBuilderRight.setCameraPose(cameraPositionRight, cameraOrientationRight);

        return this;
    }

    protected AbstractBasicDriveBot configureBot() {
        super.configureBot();

//        TBD: follower = GorillaDriveConstants.create(hardwareMap);

        return this;
    }

    static {
        cameraPositionLeft     = new Position(INCH, -6.5, 7., 10., 0);
        cameraOrientationLeft  = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);

        cameraPositionRight    = new Position(INCH,  6.5, 7., 10., 0);
        cameraOrientationRight = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);
    }

} // class GorillaDriveBot

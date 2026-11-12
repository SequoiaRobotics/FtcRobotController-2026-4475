package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.gorillacoder.AbstractBasicDriveBot.DriveMode.DRIVE_POV;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.gorillacoder.AbstractBasicDriveBot;
import org.firstinspires.ftc.gorillacoder.AbstractBasicTeleOpDriveBot;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
@Autonomous
public class GorillaPedroAutoBot extends AbstractBasicTeleOpDriveBot {

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
                updateTelemetryGameLocations();
                break;
            case TELEMETRY_APRIL_TAG_NAMES:
                updateTelemetryAprilTagNames();
                break;
        }
    }
    @Override
    protected GorillaPedroAutoBot configureAprilTagProcessors() {
        visionTask.atpBuilderLeft. setCameraPose(cameraPositionLeft,  cameraOrientationLeft);
        visionTask.atpBuilderRight.setCameraPose(cameraPositionRight, cameraOrientationRight);

        return this;
    }

    protected AbstractBasicDriveBot configureBot() {
        super.configureBot();
        return this;
    }

    static {
        cameraPositionLeft     = new Position(INCH, -6.5, 7., 10., 0);
        cameraOrientationLeft  = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);

        cameraPositionRight    = new Position(INCH,  6.5, 7., 10., 0);
        cameraOrientationRight = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);
    }

    protected double[] poses = {
        // A 2 foot circle centered 3 feet from the center of blue wall
        60, 36,   0,
        72, 48,  90,
        84, 24, 180,
        60, 36,   0
    };
    // TBD: protected PedroPathingAutoTask<AbstractBasicDriveBot> pedroTask = new PedroPathingAutoTask<AbstractBasicDriveBot>(poses);

} // class GorillaPedroAutoBot

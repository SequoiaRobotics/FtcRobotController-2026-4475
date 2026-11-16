package org.firstinspires.ftc.gorillacoder;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedro.GorillaGobildaWoodFloorDriveConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.Comparator;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.TreeSet;

@SuppressWarnings("unused")
public abstract class AbstractBasicDriveBot extends AbstractOpMode<AbstractBasicDriveBot> {

    protected abstract void readGamePad();

    public abstract static class BasicDriveTask extends AbstractBotTask<AbstractBasicDriveBot> {
    }

    protected abstract void updateTelemetry();

    protected static HashMap<String,Integer> tagsOrder        = new HashMap<>();
    protected static HashMap<String,String>  tagsRightCaption = new HashMap<>();
    protected static HashMap<String,String>  tagsLeftCaption  = new HashMap<>();

    // Initialize tagsOrder, tagsRightCaption, and tagsLeftCaption.
    {
        String tagsInOrder[] = {
            "RED AUDIENCE",   "RA",
            "RED SCORING",    "RS",
            "BLUE AUDIENCE",  "BA",
            "BLUE SCORING",   "BS",
            "Red",            "RD ",
            "Blue",           "BL",
            "Front",          "FR",
            "Back",           "BK"
        };

        for (int index = 0; index < tagsInOrder.length; index += 2) {
            String tagLong  = tagsInOrder[index];
            String tagShort = tagsInOrder[index+1];
            tagsOrder.put(tagLong, index);
            tagsRightCaption.put(tagLong, "R"+tagShort);
            tagsLeftCaption.put(tagLong, "L"+tagShort);
        }
    }

    protected String locationFrom(AprilTagClusterDetection detection) {
        if (null == detection) {
            return "x:??? y:??? b:???";
        } else {
            Pose3D   robotPose = detection.robotPose;
            Position position  = robotPose.getPosition();
            long   at      = position.acquisitionTime;
            double x       = position.x;
            double y       = position.y;
            double bearing = robotPose.getOrientation().getYaw(RADIANS);
            return String.format(Locale.US, "x:%.0f y:%.0f h:%.0f @:%d", x, y, bearing, at);
        }
    };

//    protected String bearingFrom(AprilTagClusterDetection detection) {
//        if (null == detection) {
//            return "b:??? r:??? y:???";
//        } else {
//            AprilTagPoseFtc tagPose = detection.ftcPose;
//            long   at        = detection.frameAcquisitionNanoTime;
//            double x         = tagPose.x;
//            double y         = tagPose.y;
//            double yaw       = tagPose.yaw;
//            double elevation = tagPose.elevation;
//            double bearing = tagPose.bearing;
//            return String.format(Locale.US, "x:%.0f y:%.0f b:%.0f @:%d", x, y, bearing, at);
//
//            hiveRedAudienceLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f b:%0f @:%d" ,
//                redAudienceLeftDetection.ftcPose.bearing, redAudienceLeftDetection.ftcPose.range, redAudienceLeftDetection.ftcPose.yaw);
//
//        }
//    };

    protected void updateTelemetryLocations() {
        // TO DO: Show detections for each camera/tag in the order seen, most recent first.
        // This would be the most useful order for bot localization.
        // A task would do the same, throwing out detections that are "too old".
        // Too old should be determined by encoder distance traveled since the detection.
        // Encoder distance traveled should be the some of all the encoders.
        Comparator<String> tagComparator = (left, right) -> Integer.compare(tagsOrder.get(left), tagsOrder.get(right));
        TreeSet<String> tagsAsSeen = new TreeSet<>(tagComparator);

        visionTask.targetDetectionsLeft.keySet().forEach(
            name -> tagsAsSeen.add(name)
        );
        visionTask.targetDetectionsRight.keySet().forEach(
            name -> tagsAsSeen.add(name)
        );

        telemetry.addData("Tag Position", "");
        tagsAsSeen.forEach( (String tag) -> {
            AprilTagClusterDetection detection;
            detection= this.visionTask.targetDetectionsLeft.get(tag);
            telemetry.addData( tagsLeftCaption.get(tag), locationFrom(detection) );
            detection = this.visionTask.targetDetectionsRight.get(tag);
            telemetry.addData( tagsRightCaption.get(tag), locationFrom(detection) );
        });

        telemetry.addData("Tag Bearing", "");
        tagsAsSeen.forEach( (String tag) -> {
            AprilTagClusterDetection detection;
            detection= this.visionTask.targetDetectionsLeft.get(tag);
            telemetry.addData( tagsLeftCaption.get(tag), locationFrom(detection) );
            detection = this.visionTask.targetDetectionsRight.get(tag);
            telemetry.addData( tagsRightCaption.get(tag), locationFrom(detection) );
        });
    }

    protected void updateTelemetryAprilTagNames() {
        telemetry.log().add("");
        telemetry.log().add("Left Camera AprilTag Names: ");
        visionTask.targetDetectionsLeft.keySet().forEach(
            name -> telemetry.log().add(name)
        );
        telemetry.log().add("Right Camera AprilTag Names: ");
        visionTask.targetDetectionsRight.keySet().forEach(
            name -> telemetry.log().add(name)
        );
    }

    //    protected class TelemetryTask extends TeleOpDriveTask {
    public class TelemetryTask extends BasicDriveTask {
        @Override
        public TelemetryTask run() {
            updateTelemetry();
            telemetry.update();

            return this;
        }

        @Override
        public TelemetryTask init() {
            frequencyMillis(500); // Twice per second.
            return this;
        }

        // TODO: add this to tasks, call for all tasks in AbstractOpMode.
        @Override
        public TelemetryTask waitForStart() {
            run();

            return this;
        }
    } // class TelemetryTask

    Pose pedroPoseFromRobotPose(Pose3D robotPose) {
        DistanceUnit unit = this.latestDetection.robotPose.getPosition().unit;
        double x          = unit.toInches( this.latestDetection.robotPose.getPosition().x ) ;
        double y          = unit.toInches( this.latestDetection.robotPose.getPosition().y );
        double heading    = this.latestDetection.robotPose.getOrientation().getYaw(RADIANS);

        if ( ALLIANCES_ACROSS == this.fieldConfiguration) {
            // Conversion from FTC coordinate system for a square field
            x = 72 - x;
            y = 72 + y;
            heading = -Math.PI / 2 + heading;
        } else {
            throw new RuntimeException("TO DO: Update to work with a diamond field");
        }

        Pose result = PoseFactory.radians().of(x, y, heading);
        return result;
    }

    protected AbstractBasicDriveBot configureBot() {
        telemetry.log().setCapacity(100);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        drivePovTask
            .driveLeftFront(hardwareMap.get( DcMotorEx.class, "Drive Front Left"))
            .driveRightFront(hardwareMap.get(DcMotorEx.class, "Drive Front Right"))
            .driveLeftRear(hardwareMap.get(  DcMotorEx.class, "Drive Rear Left"))
            .driveRightRear(hardwareMap.get( DcMotorEx.class, "Drive Rear Right"))
        ;
        driveTankTask
            .driveLeftFront(hardwareMap.get( DcMotorEx.class, "Drive Front Left"))
            .driveRightFront(hardwareMap.get(DcMotorEx.class, "Drive Front Right"))
            .driveLeftRear(hardwareMap.get(  DcMotorEx.class, "Drive Rear Left"))
            .driveRightRear(hardwareMap.get( DcMotorEx.class, "Drive Rear Right"))
        ;
        driveOmniTask
            .driveLeftFront(hardwareMap.get( DcMotorEx.class, "Drive Front Left"))
            .driveRightFront(hardwareMap.get(DcMotorEx.class, "Drive Front Right"))
            .driveLeftRear(hardwareMap.get(  DcMotorEx.class, "Drive Rear Left"))
            .driveRightRear(hardwareMap.get( DcMotorEx.class, "Drive Rear Right"))
        ;
        visionTask
            .cameraLeft( hardwareMap.get(WebcamName.class, "Webcam Left"))
            .cameraRight(hardwareMap.get(WebcamName.class, "Webcam Right"))
        ;

        driveMode = DriveMode.DRIVE_POV;

        if (null != this.latestDetection ) {
            follower.setPose(pedroPoseFromRobotPose(this.latestDetection.robotPose));
        }

        configurePinpoint();

        super.configureBot();
        return this;
    }

    // TO DO: Move all poinpoint nonsense to a pinpoint task.
    public void configurePinpoint(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(
            GorillaGobildaWoodFloorDriveConstants.pinpointConfig.xPodOffset.get(),
            GorillaGobildaWoodFloorDriveConstants.pinpointConfig.yPodOffset.get(),
            GorillaGobildaWoodFloorDriveConstants.pinpointConfig.offsetUnits.get()
        );

        pinpoint.setEncoderResolution(
            GorillaGobildaWoodFloorDriveConstants.pinpointConfig.podType.get()
        );

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(
            GorillaGobildaWoodFloorDriveConstants.pinpointConfig.xPodDirection.get(),
            GorillaGobildaWoodFloorDriveConstants.pinpointConfig.yPodDirection.get()
        );

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }


    protected static Position           cameraPositionLeft;
    protected static YawPitchRollAngles cameraOrientationLeft;
    protected static Position           cameraPositionRight;
    protected static YawPitchRollAngles cameraOrientationRight;

    protected VisionTaskMultiPortal<AbstractBasicDriveBot> visionTask   = new VisionTaskMultiPortal<>();

    public enum DriveMode {
        DRIVE_TANK,
        DRIVE_POV,
        DRIVE_OMNI
    }
    protected static final DriveMode DRIVE_ARCADE = DriveMode.DRIVE_POV;

    protected Map<DriveMode, AbstractDriveTask<AbstractBasicDriveBot>> driveTasks  = new HashMap<>();
    protected Map<DriveMode, DriveMode> nextDriveModeFor = new HashMap<>();

    protected GoBildaPinpointDriver                        pinpoint;

    protected BulkReadTask                                 bulkReadTask   = new BulkReadTask();
    protected DriveMode                                    driveMode      = DriveMode.DRIVE_POV;
    protected DrivePovTask<AbstractBasicDriveBot>          drivePovTask   = new DrivePovTask<>();

    protected DriveTankTask<AbstractBasicDriveBot>         driveTankTask  = new DriveTankTask<>();

    protected DriveOmniTask<AbstractBasicDriveBot>         driveOmniTask  = new DriveOmniTask<>();

    {
        nextDriveModeFor.put(DriveMode.DRIVE_TANK, DriveMode.DRIVE_POV);
        nextDriveModeFor.put(DriveMode.DRIVE_POV,  DriveMode.DRIVE_OMNI);
        nextDriveModeFor.put(DriveMode.DRIVE_OMNI, DriveMode.DRIVE_TANK);

        driveTasks.put(DriveMode.DRIVE_TANK, driveTankTask);
        driveTasks.put(DriveMode.DRIVE_POV,  drivePovTask);
        driveTasks.put(DriveMode.DRIVE_OMNI, driveOmniTask);
    }

    protected Follower follower;

} // abstract class AbstractBasicDriveBot

/*
    Blob Processing ...

    @Override
    protected GorillaDriveBot addVisionProcessors() {
        // Need to:
        // - Create the right and left portal builders and set default config. VisionTask will do this. Bots can always override.
        // - Create the left and right april tag processor builders and set default config. VisionTask will do this. Bots can always override.
        // - Create and configure other processor builders. For this game, the blob detectors.
        // - Let VisionTask.init() finish up the init, in particular create the processors and add them to the VisionPortals.
        // - Get the map from builder to processor and init our processor member variables.
        ColorBlobLocatorProcessor.Builder builderPurpleLeft = visionTask.createCircleColorBlobLocatorProcessorBuilder(ARTIFACT_PURPLE);
        ColorBlobLocatorProcessor.Builder builderGreenRight = visionTask.createCircleColorBlobLocatorProcessorBuilder(ARTIFACT_GREEN);

        BlobFilter areaFilter     = new BlobFilter(BlobCriteria.BY_CONTOUR_AREA, 20, 1000);
        BlobFilter densityFilter  = new BlobFilter(BlobCriteria.BY_DENSITY,      0.3, 1.0);
        BlobFilter ratioFilter    = new BlobFilter(BlobCriteria.BY_ASPECT_RATIO, 1.0, 2);
        BlobSort   blobSortByArea = new BlobSort(BlobCriteria.BY_CONTOUR_AREA,   SortOrder.ASCENDING);

        blobLocatorPurpleLeft = builderPurpleLeft
                // Smooth the transitions between different colors in image
                .setBlurSize(5)
                // fill in perimeter holes. Dilate to fill in edge divots, then shrink to original size.
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .setDilateSize(15)
                .setErodeSize(15)
                .build();
        blobLocatorGreenRight = builderGreenRight
                // Smooth the transitions between different colors in image
                .setBlurSize(5)
                // fill in perimeter holes. Dilate to fill in edge divots, then shrink to original size.
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .setDilateSize(15)
                .setErodeSize(15)
                .build();

        blobLocatorPurpleLeft.addFilter(areaFilter);
        blobLocatorPurpleLeft.addFilter(densityFilter);
        blobLocatorPurpleLeft.addFilter(ratioFilter);
        blobLocatorPurpleLeft.setSort(blobSortByArea);

        blobLocatorGreenRight.addFilter(areaFilter);
        blobLocatorGreenRight.addFilter(densityFilter);
        blobLocatorGreenRight.addFilter(ratioFilter);
        blobLocatorGreenRight.setSort(blobSortByArea);

        visionTask.addProcessorLeft(blobLocatorPurpleLeft);
        visionTask.addProcessorRight(blobLocatorGreenRight);
    }
 */
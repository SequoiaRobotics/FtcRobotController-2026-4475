package org.firstinspires.ftc.gorillacoder;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
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

    protected void updateTelemetryGameLocations() {
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

        telemetry.addData("Position", "");
        tagsAsSeen.forEach( (String tag) -> {
            AprilTagClusterDetection detection;
            detection= this.visionTask.targetDetectionsLeft.get(tag);
            telemetry.addData( tagsLeftCaption.get(tag), locationFrom(detection) );
            detection = this.visionTask.targetDetectionsRight.get(tag);
            telemetry.addData( tagsRightCaption.get(tag), locationFrom(detection) );
        });

        telemetry.addData("Bearing", "");
        tagsAsSeen.forEach( (String tag) -> {
            AprilTagClusterDetection detection;
            detection= this.visionTask.targetDetectionsLeft.get(tag);
            telemetry.addData( tagsLeftCaption.get(tag), locationFrom(detection) );
            detection = this.visionTask.targetDetectionsRight.get(tag);
            telemetry.addData( tagsRightCaption.get(tag), locationFrom(detection) );
        });
    }

    protected void updateTelemetryGameLocations0() {


        String BlueAudienceName = "BLUE AUDIENCE";
        String BlueScoringName  = "BLUE SCORING";
        String RedAudienceName  = "RED AUDIENCE";
        String RedScoringName   = "RED SCORING";

        // Red Hive seen from left camera
        AprilTagClusterDetection redAudienceLeftDetection   = visionTask.targetDetectionsLeft.get(RedAudienceName);
        String                   hiveRedAudienceLeft        = "b:? r:? y:?";
        String                   botRedAudienceLeft         = "x:? y:? y:?";
        AprilTagClusterDetection redScoringLeftDetection    = visionTask.targetDetectionsLeft.get(RedScoringName);
        String                   hiveRedScoringLeft         = "b:? r:? y:?";
        String                   botRedScoringLeft          = "x:? y:? y:?";

        if (null != redAudienceLeftDetection) {
            hiveRedAudienceLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                redAudienceLeftDetection.ftcPose.bearing, redAudienceLeftDetection.ftcPose.range, redAudienceLeftDetection.ftcPose.yaw);
            botRedAudienceLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                redAudienceLeftDetection.robotPose.getPosition().x, redAudienceLeftDetection.robotPose.getPosition().y, redAudienceLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
        }
        if (null != redScoringLeftDetection) {
            hiveRedScoringLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                redScoringLeftDetection.ftcPose.bearing, redScoringLeftDetection.ftcPose.range, redScoringLeftDetection.ftcPose.yaw);
            botRedScoringLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                redScoringLeftDetection.robotPose.getPosition().x, redScoringLeftDetection.robotPose.getPosition().y, redScoringLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
        }

        // Red Hive seen from right camera
        AprilTagClusterDetection redAudienceRightDetection  = visionTask.targetDetectionsRight.get(RedAudienceName);
        String                   hiveRedAudienceRight       = "b:? r:? y:?";
        String                   botRedAudienceRight        = "x:? y:? y:?";
        AprilTagClusterDetection redScoringRightDetection   = visionTask.targetDetectionsRight.get(RedScoringName);
        String                   hiveRedScoringRight        = "b:? r:? y:?";
        String                   botRedScoringRight         = "x:? y:? y:?";

        if (null != redAudienceRightDetection) {
            hiveRedAudienceRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                redAudienceRightDetection.ftcPose.bearing, redAudienceRightDetection.ftcPose.range, redAudienceRightDetection.ftcPose.yaw);
            botRedAudienceRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                redAudienceRightDetection.robotPose.getPosition().x, redAudienceRightDetection.robotPose.getPosition().y, redAudienceRightDetection.robotPose.getOrientation().getYaw(DEGREES));
        }
        if (null != redScoringRightDetection) {
            hiveRedScoringRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                redScoringRightDetection.ftcPose.bearing, redScoringRightDetection.ftcPose.range, redScoringRightDetection.ftcPose.yaw);
            botRedScoringRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                redScoringRightDetection.robotPose.getPosition().x, redScoringRightDetection.robotPose.getPosition().y, redScoringRightDetection.robotPose.getOrientation().getYaw(DEGREES));
        }

        // Blue Hive seen from left camera
        AprilTagClusterDetection blueAudienceLeftDetection  = visionTask.targetDetectionsLeft.get(BlueAudienceName);
        String                   hiveBlueAudienceLeft       = "b:? r:? y:?";
        String                   botBlueAudienceLeft        = "x:? y:? y:?";
        AprilTagClusterDetection blueScoringLeftDetection   = visionTask.targetDetectionsLeft.get(BlueScoringName);
        String                   hiveBlueScoringLeft        = "b:? r:? y:?";
        String                   botBlueScoringLeft         = "x:? y:? y:?";

        if (null != blueAudienceLeftDetection) {
            hiveBlueAudienceLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                blueAudienceLeftDetection.ftcPose.bearing, blueAudienceLeftDetection.ftcPose.range, blueAudienceLeftDetection.ftcPose.yaw);
            botBlueAudienceLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                blueAudienceLeftDetection.robotPose.getPosition().x, blueAudienceLeftDetection.robotPose.getPosition().y, blueAudienceLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
        }
        if (null != blueScoringLeftDetection) {
            hiveBlueScoringLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                blueScoringLeftDetection.ftcPose.bearing, blueScoringLeftDetection.ftcPose.range, blueScoringLeftDetection.ftcPose.yaw);
            botBlueScoringLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                blueScoringLeftDetection.robotPose.getPosition().x, blueScoringLeftDetection.robotPose.getPosition().y, blueScoringLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
        }

        // Blue Hive seen from right camera
        AprilTagClusterDetection blueAudienceRightDetection = visionTask.targetDetectionsRight.get(BlueAudienceName);
        String                   hiveBlueAudienceRight      = "b:? r:? y:?";
        String                   botBlueAudienceRight       = "x:? y:? y:?";
        AprilTagClusterDetection blueScoringRightDetection  = visionTask.targetDetectionsRight.get(BlueScoringName);
        String                   hiveBlueScoringRight       = "b:? r:? y:?";
        String                   botBlueScoringRight        = "x:? y:? y:?";

        if (null != blueAudienceRightDetection) {
            hiveBlueAudienceRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                blueAudienceRightDetection.ftcPose.bearing, blueAudienceRightDetection.ftcPose.range, blueAudienceRightDetection.ftcPose.yaw);
            botBlueAudienceRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                blueAudienceRightDetection.robotPose.getPosition().x, blueAudienceRightDetection.robotPose.getPosition().y, blueAudienceRightDetection.robotPose.getOrientation().getYaw(DEGREES));
        }
        if (null != blueScoringRightDetection) {
            hiveBlueScoringRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                blueScoringRightDetection.ftcPose.bearing, blueScoringRightDetection.ftcPose.range, blueScoringRightDetection.ftcPose.yaw);
            botBlueScoringRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                blueScoringRightDetection.robotPose.getPosition().x, blueScoringRightDetection.robotPose.getPosition().y, blueScoringRightDetection.robotPose.getOrientation().getYaw(DEGREES));
        }

        telemetry.addData("Bot RSL", "%s", botRedScoringLeft);
        telemetry.addData("    RSR", "%s", botRedScoringRight);
        telemetry.addData("    RAL", "%s", botRedAudienceLeft);
        telemetry.addData("    RAR", "%s", botRedAudienceRight);
        telemetry.addData("    BSL", "%s", botBlueScoringLeft);
        telemetry.addData("    BSR", "%s", botBlueScoringRight);
        telemetry.addData("    BAL", "%s", botBlueAudienceLeft);
        telemetry.addData("    BAR", "%s", botBlueAudienceRight);
        telemetry.addData("Hiv RSL", "%s", hiveRedScoringLeft);
        telemetry.addData("    RSR", "%s", hiveRedScoringRight);
        telemetry.addData("    RAL", "%s", hiveRedAudienceLeft);
        telemetry.addData("    RAR", "%s", hiveRedAudienceRight);
        telemetry.addData("    BSL", "%s", hiveBlueScoringLeft);
        telemetry.addData("    BSR", "%s", hiveBlueScoringRight);
        telemetry.addData("    BAL", "%s", hiveBlueAudienceLeft);
        telemetry.addData("    BAR", "%s", hiveBlueAudienceRight);
    }

    protected void updateTelemetryTestLocations() {

        String redTagName    = "Red";
        String blueTagName   = "Blue";
        String frontTagName  = "Front";
        String backTagName   = "Back";

        // Red Tag seen from left camera
        AprilTagClusterDetection redLeftDetection   = visionTask.targetDetectionsLeft.get(redTagName);
        String redTagLeft    = "b:? r:? y:?";
        String botRedTagLeft = "x:? y:? y:?";

        if (null != redLeftDetection) {
            redTagLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                redLeftDetection.ftcPose.bearing, redLeftDetection.ftcPose.range, redLeftDetection.ftcPose.yaw);
            botRedTagLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                redLeftDetection.robotPose.getPosition().x, redLeftDetection.robotPose.getPosition().y, redLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
        }

        // Blue Tag seen from left camera
        AprilTagClusterDetection blueLeftDetection = visionTask.targetDetectionsLeft.get(redTagName);
        String blueTagLeft = "b:? r:? y:?";
        String botBlueTagLeft = "x:? y:? y:?";

        if (null != blueLeftDetection) {
            blueTagLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                blueLeftDetection.ftcPose.bearing, blueLeftDetection.ftcPose.range, blueLeftDetection.ftcPose.yaw);
            botBlueTagLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                blueLeftDetection.robotPose.getPosition().x, blueLeftDetection.robotPose.getPosition().y, blueLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
        }

        telemetry.addData("Bot RL", "%s", botRedTagLeft);
        telemetry.addData("    BL", "%s", botBlueTagLeft);
        telemetry.addData("Tag RL", "%s", redTagLeft);
        telemetry.addData("    BL", "%s", blueTagLeft);
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

        super.configureBot();
        return this;
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
package org.firstinspires.ftc.gorillacoder;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.Blob;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobCriteria;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobFilter;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobSort;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.Locale;

@SuppressWarnings("unused")
public abstract class AbstractBasicDriveBot extends AbstractOpMode<AbstractBasicDriveBot> {

    public abstract static class TeleOpDriveTask extends AbstractBotTask<AbstractBasicDriveBot> {
    }

    protected abstract void readGamePad();

    protected class GamePadTask extends TeleOpDriveTask {
        @Override
        public GamePadTask run() {
           readGamePad();

            return this;
        }

        @Override
        public GamePadTask init() {
            frequencyMillis(5); // Every 5 ms, 200/second.
            return this;
        }
    } // class GamePadTask

    protected abstract void updateTelemetry();

//    protected class TelemetryTask extends TeleOpDriveTask {
    public class TelemetryTask extends TeleOpDriveTask {
        @Override
        public TelemetryTask run() {
            updateTelemetry();
            telemetry.update();

            return this;
        }

        @Override
        public TelemetryTask init() {
            frequencyMillis(100); // Every 5 ms, 200/second.
            return this;
        }

        // TODO: add this to tasks, call for all tasks in AbstractOpMode.
        public TelemetryTask waitForStart() {
            run();

            return this;
        }
    } // class TelemetryTask

    protected AbstractBasicDriveBot configureTelemetry() {
        telemetry.log().setCapacity(100);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        return this;
    }

    @Override
    protected BotTask<AbstractBasicDriveBot>[] getTasks() {
        telemetry.addData("status", "TeleOpDrive.createTasks(): tasks created");

        drivePovTask
            .driveLeftFront(hardwareMap.get( DcMotorEx.class, "Drive Front Left"))
            .driveRightFront(hardwareMap.get(DcMotorEx.class, "Drive Front Right"))
            .driveLeftRear(hardwareMap.get(  DcMotorEx.class, "Drive Rear Left"))
            .driveRightRear(hardwareMap.get( DcMotorEx.class, "Drive Rear Right"))
        ;
        visionTask
            .cameraLeft( hardwareMap.get(WebcamName.class, "Webcam Left"))
            .cameraRight(hardwareMap.get(WebcamName.class, "Webcam Right"));
        telemetry.addData("status", "TeleOpDrive.createTasks(): tasks connected to hardware");

        @SuppressWarnings("unchecked")
        BotTask<AbstractBasicDriveBot>[] result = new BotTask[] {
            visionTask,
            drivePovTask,
//                driveTankTask,
            gamePadTask,
            new TelemetryTask()
        };
        return result;
    }

    protected static Position           cameraPositionLeft;
    protected static YawPitchRollAngles cameraOrientationLeft;
    protected static Position           cameraPositionRight;
    protected static YawPitchRollAngles cameraOrientationRight;

    protected VisionTaskMultiPortal<AbstractBasicDriveBot> visionTask   = new VisionTaskMultiPortal<>();

    protected DrivePovTask<AbstractBasicDriveBot>          drivePovTask = new DrivePovTask<>();

    protected DriveTankTask<AbstractBasicDriveBot>         driveTankTask = new DriveTankTask<>();

    protected GamePadTask                                  gamePadTask   = new GamePadTask();

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
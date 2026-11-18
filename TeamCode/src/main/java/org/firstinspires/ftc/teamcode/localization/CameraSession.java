package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// Groups each camera's vision processor with its portal.
public class CameraSession {
    public final String name;
    public final AprilTagProcessor processor;
    public final VisionPortal portal;

    CameraSession(String name, AprilTagProcessor processor, VisionPortal portal) {
        this.name = name;
        this.processor = processor;
        this.portal = portal;
    }


    public static CameraSession initializeCamera(
            HardwareMap hardwareMap,
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
}
//
//  FTCDrivebot.java
//  Robotics
//
//  Created by Nikhil Rasiah on 9/22/26.
//
// IMPORTS FROM FTC
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;
import android.util.Size;



@TeleOp
public class FTCDrivebot extends LinearOpMode {

  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;
  
  @Override
  public void runOpMode() {
    // Initialize hardware variables (motors)
    DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
    DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
    DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
    DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

    // Reverse left motors
    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    // Set encoders
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    // PLACEHOLDERS — replace after mounting the camera.
    // Position in inches relative to the robot's chosen origin:
    double cameraRight = 9;
    double cameraForward = 9;
    double cameraUp = 5;
    
    // Assumes an upright camera facing forward horizontally:
    double cameraYaw = 0;
    double cameraPitch = -90;
    double cameraRoll = 0;

    Position cameraPosition = new Position(
      DistanceUnit.INCH,
      cameraRight, cameraForward, cameraUp,
      0
    );

    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
        AngleUnit.DEGREES,
        cameraYaw, cameraPitch, cameraRoll,
        0
    );

    // Create april tag processor + vision portal
    aprilTag = new AprilTagProcessor.Builder()
    .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
    .setCameraPose(cameraPosition, cameraOrientation)
    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
    .build();
    
    // Initialize the vision portal
    visionPortal = new VisionPortal.Builder()
      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
      .addProcessor(aprilTag)
      .setCameraResolution(new Size(640, 480))
      .build();
    
    waitForStart();

    while (opModeIsActive()) {
      double forward = -gamepad1.left_stick_y; 
      double strafe = gamepad1.left_stick_x;
      double turn = gamepad1.right_stick_x;

      // Joy stick softening
      forward = Math.copySign(forward * forward, forward);
      strafe  = Math.copySign(strafe * strafe, strafe);
      turn    = Math.copySign(turn * turn, turn);
            
      // Calculate raw motor powers
      double frontLeftPower  = forward + strafe + turn;
      double backLeftPower   = forward - strafe + turn;
      double frontRightPower = forward - strafe - turn;
      double backRightPower  = forward + strafe - turn;
      
      // Find the largest power value to normalize if any exceed 1.0
      double max = Math.max(1.0, Math.max(
          Math.abs(frontLeftPower), Math.max(
          Math.abs(backLeftPower), Math.max(
          Math.abs(frontRightPower), Math.abs(backRightPower))))
      );
      
      frontLeftPower  /= max;
      backLeftPower   /= max;
      frontRightPower /= max;
      backRightPower  /= max;
      frontLeftMotor.setPower(frontLeftPower);
      backLeftMotor.setPower(backLeftPower);
      frontRightMotor.setPower(frontRightPower);
      backRightMotor.setPower(backRightPower);

      List<AprilTagDetection> currentDetections = aprilTag.getDetections();
      telemetry.addData("# Detections", currentDetections.size());
      
      for (AprilTagDetection detection : currentDetections) {
          // This OpMode displays BIOBUZZ clusters.
          if (!(detection instanceof AprilTagClusterDetection)) {
              continue;
          }
      
          AprilTagClusterDetection cluster =
              (AprilTagClusterDetection) detection;

          // Add telemetry, and diagnose any obstructions
          telemetry.addData("Cluster", cluster.metadata.name);
          telemetry.addData("Tags visible", "%d%%", cluster.percentClusterFound);
      
          if (cluster.ftcPose == null) {
              telemetry.addLine("Pose unavailable");
              continue;
          }

          // Cell center relative to camera (position)
          telemetry.addData(
              "Opening relative to CAMERA (in)",
              "X %.1f | Y %.1f | Z %.1f",
              cluster.ftcPose.x,
              cluster.ftcPose.y,
              cluster.ftcPose.z
          );

          // Cell center relative to camera (pose)
          telemetry.addData(
              "Orientation (deg)",
              "Pitch %.1f | Roll %.1f | Yaw %.1f",
              cluster.ftcPose.pitch,
              cluster.ftcPose.roll,
              cluster.ftcPose.yaw
          );
      }
      
      telemetry.update();
    }
    // Clean up resources when done
    visionPortal.close();
  }
}

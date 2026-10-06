//
//  DriveTrain.java
//  Robotics
//
//  Created by Nikhil Rasiah on 9/22/26.
//
// IMPORTS FROM FTC
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
  
    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    // Create april tag processor + vision portal
    aprilTag = new AprilTagProcessor.Builder()
      .build();

    // Initialize the vision portal
    //visionPortal = new VisionPortal.Builder()
    //  .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    //  .addProcessor(aprilTag)
    //  .SetCameraResolution(new Size(640, 480))
    //  .build();
    
    waitForStart();

    while (opModeIsActive()) {
      double forward = -gamepad1.left_stick_y;
      double turn = -gamepad1.right_stick_x;
      double strafe = -gamepad1.left_stick_x;

      double frontLeft = forward + turn + strafe;
      double frontRight = forward - turn - strafe;
      double backLeft = forward + turn - strafe;
      double backRight = forward - turn + strafe;
      

      double scale = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight))));

      frontLeft /= scale;
      frontRight /= scale;
      backLeft /= scale;
      backRight /= scale;

      frontLeftMotor.setPower(frontLeft);
      backLeftMotor.setPower(backLeft);
      frontRightMotor.setPower(frontRight);
      backRightMotor.setPower(backRight);

      //List<AprilTagDetections> currentDetections = aprilTag.getDetections();
      //telemetry.addData("# April tags detected", currentDetections.size());
      
      // Display details for detected april tags
      //for(AprilTagDetection detection : currentDetections) {
      //  if (detection.metadata != null) {
      //    telemetry.addLine(String.format("\n==== (ID %d) %s ====", detection.id, detection.metadata.name));
      //    telemetry.addLine(String.format("XYZ: %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
      //    telemetry.addLine(String.format("PRY: %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
      //  } else {
      //    telemetry.addLine(String.format("\n==== (ID %d) Unknown Tag ====", detection.id));
      //  }
      //}
      // Update new telemetry
      //telemetry.update();
    }
    // Clean up resources when done
    //visionPortal.close();
  }
}

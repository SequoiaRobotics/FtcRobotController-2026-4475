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

@TeleOp
public class DriveTrain extends LinearOpMode {

  
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

    visionPortal = new VisionPortal.Builder()
      .build();
  
    waitForStart();

    while (opModeIsActive()) {
      double forward = -gamepad1.left_stick_y;
      double turn = -gamepad1.right_stick_x;

      double leftPower = forward + turn;
      double rightRower = forward - turn;

      double scale = Math.max(1.0, Math.max(Math.Abs(leftPower), Math.Abs(rightPower));

      leftPower /= scale;
      rightPower /= scale;
      frontLeftMotor.setPower(leftPower);
      backLeftMotor.setPower(leftPower);
      frontRightMotor.setPower(rightPower);
      backRightMotor.setPower(rightPower);
    }
  }

  


  
  
}

package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

// Omni Mode uses one stick to control forward and strafe and the other stick to control rotation.
public class DriveOmniTask<OpModeT extends AbstractOpMode<OpModeT>> extends AbstractDriveTask<OpModeT>  {

    @Override
    public DriveOmniTask<OpModeT> run() {
        double max;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        RobotLog.ii(
            AbstractOpMode.GORILLA_CORE, "%s.run(): axial:%f lateral:%f yaw:%f",
            getClass().getSimpleName(), axial, lateral, yaw
        );

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

//        leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//        leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//        rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//        rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

        // Send calculated power to wheels
        driveLeftFront.setPower(leftFrontPower);
        driveRightFront.setPower(rightFrontPower);
        driveLeftRear.setPower(leftBackPower);
        driveRightRear.setPower(rightBackPower);

        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() done: leftFront:%f rightFront:%f leftRear:%f rightRear:%f",
            getClass().getSimpleName(), leftFrontPower, rightFrontPower, leftBackPower, rightBackPower
        );

        return this;
    }

    @Override
    public DriveOmniTask<OpModeT> init() {
        super.init();
        this.frequencyMillis(10); // adjust motors every 10ms, 100/second
        return this;
    }

    public double axial = 0.0;

    public double lateral = 0.0;

    public double yaw = 0.0;

} // class DriveOmniTask

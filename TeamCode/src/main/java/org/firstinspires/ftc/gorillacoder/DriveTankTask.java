package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

// Tank Mode uses one stick to control each side of wheels.
@SuppressWarnings("unused")
public class DriveTankTask<OpModeT extends AbstractOpMode<OpModeT>> extends AbstractDriveTask<OpModeT>  {

    @Override
    public DriveTankTask<OpModeT> run() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() start: left:%f right:%f", getClass().getSimpleName(), leftPower, rightPower);

        // Send calculated power to wheels
        driveLeftRear.setPower(leftPower);
        driveLeftFront.setPower(leftPower);
        driveRightRear.setPower(rightPower);
        driveRightFront.setPower(rightPower);

        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() done: left:%f right:%f", getClass().getSimpleName(), leftPower, rightPower);
        return this;
    }

    @Override
    public DriveTankTask<OpModeT> init() {
        super.init();
        this.frequencyMillis(10); // adjust motors every 10ms, 100/second
        return this;
    }

    public double leftPower;

    public double rightPower;

} // class DriveTankTask

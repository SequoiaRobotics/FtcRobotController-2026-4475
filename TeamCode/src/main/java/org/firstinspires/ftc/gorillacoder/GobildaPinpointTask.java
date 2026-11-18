package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.GorillaGobildaWoodFloorDriveConstants;

// GobildaPinpointTask manages Gobilda Pinpoint
@SuppressWarnings("unused")
public class GobildaPinpointTask<OpModeT extends AbstractOpMode<OpModeT>> extends AbstractOpMode.AbstractOpModeTask<OpModeT> {

    @Override
    public GobildaPinpointTask<OpModeT> run() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() start", getClass().getSimpleName());
        pinpoint.update();
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() done", getClass().getSimpleName());
        return this;
    }

    @Override
    public GobildaPinpointTask<OpModeT> init() {
        super.init();
        this.frequencyMillis(100); // reset every 100ms, 10/second. Is this too often?

        this.pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        return this;
    }

    public Pose2D getPosition() {
        if (null == pinpoint) {
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        }
        return pinpoint.getPosition();
    }

    public void configurePinpoint(){
        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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

    protected GoBildaPinpointDriver pinpoint;

} // class GobildaPinpointTask




package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flywheel Velocity Test")
public class FlywheelVelocityTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotorEx flywheel = hardwareMap.get(
                DcMotorEx.class, "flywheelMotor"
        );

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);

        double power = 0.5;
        boolean running = false;
        double maxVelocity = 0;

        try {
            telemetry.addLine("X: Start / stop");
            telemetry.addLine("D-pad Up/Down: Adjust power");
            telemetry.addLine("CIRC: Force stop");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {

                if (gamepad1.aWasPressed()) {
                    running = !running;
                }

                if (gamepad1.bWasPressed()) {
                    running = false;
                }

                if (gamepad1.dpadUpWasPressed()) {
                    power = Math.min(1.0, power + 0.1);
                }

                if (gamepad1.dpadDownWasPressed()) {
                    power = Math.max(0.0, power - 0.1);
                }

                flywheel.setPower(running ? power : 0);

                double velocity = Math.abs(flywheel.getVelocity());

                if (running) {
                    maxVelocity = Math.max(maxVelocity, velocity);
                }

                telemetry.addData("Running", running);
                telemetry.addData("Power", "%.2f", power);
                telemetry.addData("Velocity (ticks/s)", "%.1f", velocity);
                telemetry.addData("Max observed (ticks/s)", "%.1f",
                        maxVelocity);
                telemetry.update();

                sleep(20);
            }

        } finally {
            flywheel.setPower(0);
        }
    }
}


package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    // ============== CONFIGURATION ============== //

    private final DcMotor intakeMotor;

    // ENABLE / CONSTRUCT IF THIRD AXEL DOESN'T FEED
    // private final DcMotor thirdAxelIntakeMotor;

    // TODO: Adjust power after testing.
    private static final double INTAKE_POWER = 0.8;
    private static final double INTAKE_REVERSE_POWER = -0.3;
    // TODO: Adjust intake duration after testing.
    public static final double COLLECT_DURATION = 1.5;

    private boolean running = false;


    // ============== INITIALIZATION ============== //

    public Intake(HardwareMap hardwareMap) {

        // Retrieves the motor configured on Expansion Hub port 0.
        // "intakeMotor" must match the Robot Configuration name.
        intakeMotor = hardwareMap.get(
                DcMotor.class,
                "intakeMotor"
        );
        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        intakeMotor.setPower(0);


    }


    // ============== START INTAKE ============== //

    public void start() {
        running = true;
        intakeMotor.setPower(INTAKE_POWER);
        // thirdAxelIntakeMotor.setPower(INTAKE_POWER);
    }


    // ============== STOP INTAKE ============== //

    public void stop() {
        running = false;
        intakeMotor.setPower(0);
        // thirdAxelIntakeMotor.setPower(0);
    }

    // =============== REVERSE INTAKE =============== //
    public void reverse() {
        running = true;
        intakeMotor.setPower(INTAKE_REVERSE_POWER);
    }


    // ============== STATUS ============== //

    public boolean isRunning() {
        return running;
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretStationary {

    // ====== HARDWARE ====== //

    private final DcMotorEx flywheelMotor;
    private final DcMotor indexer;

    private static final String FLYWHEEL_NAME = "flywheelMotor";
    private static final String BLOCKER_NAME = "blockerServo";

    // ====== TUNABLE CONSTANTS ====== //

    private static final double FLYWHEEL_POWER = 1.0;
    private static final double INDEXER_POWER = 1.0;

    private static final double BLOCKER_OPEN = 0.0;   // TODO: Calibrate
    private static final double BLOCKER_CLOSED = 0.0; // TODO: Calibrate

    public static final double FEEDING_DURATION = 3.0;

    public static final boolean USING_FLYWHEEL_ENCODER = true;

    public static final double TARGET_VELOCITY = 1700;
    public static final double VELOCITY_TOLERANCE = 100;
    private static final double SPINUP_DURATION = 2.0;

    private boolean flywheelRunning = false;
    private boolean flywheelReady = false;

    // ====== STATE MACHINE ====== //

    public enum TurretState {
        IDLE,
        SPINNING_UP,
        READY,
        FEEDING
    }

    private TurretState turretState = TurretState.IDLE;
    private long stateStartTime;

    public double secondsInState() {
        return (System.nanoTime() - stateStartTime) / 1e9;
    }

    public void setState(TurretState newState) {
        turretState = newState;
        stateStartTime = System.nanoTime();
    }

    // ====== INITIALIZATION ====== //

    public TurretStationary(HardwareMap hardwareMap) {

        // Flywheel motor restored
        flywheelMotor = hardwareMap.get(
                DcMotorEx.class, FLYWHEEL_NAME
        );

        if (USING_FLYWHEEL_ENCODER) {
            flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        flywheelMotor.setPower(0);
        flywheelMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        // Indexer motor
        indexer = hardwareMap.get(
                DcMotor.class, "indexerMotor"
        );

        indexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        indexer.setPower(0);

        indexer.setDirection(DcMotor.Direction.REVERSE);
    }

    // ====== UPDATE STATE ====== //

    public void update() {

        if (turretState != TurretState.SPINNING_UP) {
            return;
        }

        if (USING_FLYWHEEL_ENCODER) {

            double velocity = Math.abs(flywheelMotor.getVelocity());

            if (TARGET_VELOCITY > 0 &&
                    Math.abs(velocity - TARGET_VELOCITY)
                            <= VELOCITY_TOLERANCE) {

                setState(TurretState.READY);
                flywheelReady = true;
            }

        } else {

            if (flywheelRunning &&
                    secondsInState() >= SPINUP_DURATION) {

                setState(TurretState.READY);
                flywheelReady = true;
            }
        }
    }

    // ====== FLYWHEEL CONTROL ====== //

    public void startFlyWheel() {

        if (turretState != TurretState.IDLE) {
            return;
        }

        if (USING_FLYWHEEL_ENCODER) {
            flywheelMotor.setVelocity(TARGET_VELOCITY);
        } else {
            flywheelMotor.setPower(FLYWHEEL_POWER);
        }

        flywheelRunning = FLYWHEEL_POWER != 0.0;
        flywheelReady = false;

        setState(TurretState.SPINNING_UP);
    }

    public void stopFlyWheel() {

        flywheelMotor.setPower(0);

        flywheelRunning = false;
        flywheelReady = false;

        setState(TurretState.IDLE);
    }

    public boolean isFlywheelRunning() {
        return flywheelRunning;
    }

    public boolean isFlywheelReady() {
        return flywheelReady;
    }

    // ====== INDEXER / BLOCKER CONTROL ====== //

    public void indexerOn() {

        // Temporarily feed pollen using indexer motor.
        indexer.setPower(INDEXER_POWER);

        // blockerServo.setPosition(BLOCKER_OPEN);
        // setState(TurretState.FEEDING);
    }

    public void indexerReverse() {
        indexer.setPower((-INDEXER_POWER));
    }

    public void indexerOff() {

        indexer.setPower(0);

        // blockerServo.setPosition(BLOCKER_CLOSED);
    }

    // ====== SHUTDOWN ====== //

    public void stop() {
        indexerOff();
        stopFlyWheel();
    }
}
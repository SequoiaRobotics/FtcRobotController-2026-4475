
package org.firstinspires.ftc.teamcode.subsystems;

public class Shooter {

    // ============== HARDWARE CONFIGURATION ============== //

    // TODO: Declare flywheel motor(s), feeder motor/servo,
    // and any sensors used to detect Pollen or flywheel speed.
    // TODO: Define target flywheel velocity and feed settings.

    // Placeholder state. Does NOT represent actual motor readiness.
    private boolean ready = false;

    // Number of shots requested, NOT successfully fired.
    private int requestedShots = 0;


    // ============== INITIALIZATION ============== //

    // TODO: Add a constructor accepting HardwareMap.
    // Initialize motors, encoders, and feeder mechanisms here.
    // Ensure the shooter starts in a safe, inactive state.


    // ============== PREPARE SHOOTER ============== //

    public void prepare() {
        ready = true;

        // TODO: Start flywheel motors at the target velocity.
        // Actual readiness should depend on measured flywheel
        // speed, not simply calling prepare().
    }


    // ============== REQUEST SHOTS ============== //

    public void shoot(int count) {
        requestedShots += count;

        // TODO: Implement a nonblocking feeder sequence.
        // Check shooter readiness before feeding Pollen.
        // Track completed shots separately from requested shots.
    }


    // ============== READINESS CHECK ============== //

    public boolean isReady() {
        // TODO: Eventually check measured flywheel velocity
        // and any required feeder/safety conditions.
        return ready;
    }


    // ============== SHOT COUNTER ============== //

    public int getRequestedShots() {
        return requestedShots;
    }


    // ============== CONTINUOUS UPDATE ============== //

    public void update() {
        // TODO: Implement flywheel velocity feedback control,
        // feeder timing, and shot-completion detection.
        // Called once per OpMode loop.
    }


    // ============== SHUTDOWN ============== //

    public void stop() {
        ready = false;

        // TODO: Stop flywheel motors, disable the feeder,
        // and cancel any pending shot requests safely.
    }
}

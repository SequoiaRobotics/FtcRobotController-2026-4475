
package org.firstinspires.ftc.teamcode.subsystems;

public class Intake {

    // ============== HARDWARE CONFIGURATION ============== //

    // TODO: Declare intake motor(s) and any collection sensors.
    // TODO: Define intake power, reverse power, and motor directions.
    // TODO: Configure sensor thresholds for detecting Pollen.


    // ============== INTERNAL STATE ============== //

    // Tracks whether the intake has been commanded to run.
    // Does NOT confirm that a motor is physically spinning.
    private boolean running = false;


    // ============== INITIALIZATION ============== //

    // TODO: Add a constructor accepting HardwareMap.
    // Initialize motors and sensors using their configuration names.
    // Configure motor directions and zero-power behavior.


    // ============== START INTAKE ============== //

    public void start() {
        running = true;

        // TODO: Set intake motor(s) to the desired power.
        // Keep this method nonblocking so Pedro can keep updating.
    }


    // ============== STOP INTAKE ============== //

    public void stop() {
        running = false;

        // TODO: Set intake motor(s) to zero power.
    }


    // ============== STATUS CHECK ============== //

    public boolean isRunning() {
        return running;
    }


    // ============== FUTURE COLLECTION DETECTION ============== //

    // TODO: Add hasPollen() using a distance, beam-break,
    // or another suitable sensor.
    //
    // Autonomous can use this to decide when collection is complete.
    // For multiple Pollen, consider tracking how many are stored.

    // TODO: Consider adding:
    // - Reverse/unjam control
    // - Jam detection
    // - Collection timeout (handled by autonomous or subsystem)
    // - Maximum capacity detection


    // ============== FUTURE CONTINUOUS UPDATE ============== //

    // TODO: Add update() if the intake needs ongoing sensor
    // monitoring, jam handling, or a timed collection sequence.
    // Call it once per OpMode loop if implemented.
}

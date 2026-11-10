package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.teamcode.localization.CameraOdometryCorrector;
import org.firstinspires.ftc.teamcode.localization.HiveOrientationDetector;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;

// Subsystems
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import android.text.AutoText;

@Autonomous
public class AutonomousPurpleReign {

    // Path follower
    private Follower follower;

    // Subsystems
    private Intake intake;
    private Turret turret;
    private Shooter shooter;
    // Localization
    private CameraOdometryCorrector cameraCorrector;
    private HiveOrientationDetector hiveOrientationDetector;

    // Pose factory
    private final PoseFactory p = PoseFactory.degrees();


    //============== EDITABLE FIELD POSITIONS ============//

    // Edit based on where the robot starts on the field
    private final Pose bottomStartPose   = p.of(58, 7.75, 90);
    private final Pose topStartPose = p.of(58, 136.25, 270);

    // Edit based on how close to the hive the turret must be, and the orientation of the turret.
    private final Pose scorePoseBottom   = p.of(58, 20, 155);
    private final Pose scorePoseTop = p.of(58, 110, 270);

    // Edit based on required flower collection position
    private final Pose collectPose = p.of(12, 47, 180);

    // Placeholder control point for collect --> top score. Remove / edit if needed
    private final Pose controlCollectToTop = p.of(20, 110, 0);

    // Edit based on final parking position
    private final Pose upperParkPose = p.of(8, 118, 180);
    private final Pose lowerParkPose = p.of(8, 97, 180);




    // State machine for autonomous
    private enum AutoState {
        DRIVE_TO_BOTTOM_SCORE,
        SCORE_BOTTOM,
        DRIVE_TO_COLLECT,
        COLLECT,
        DRIVE_TO_TOP_SCORE,
        SCORE_TOP,
        DRIVE_TO_PARK,
        FINISHED
    }

    private AutoState autoState = AutoState.DRIVE_TO_BOTTOM_SCORE;
    private long stateStartTime;

    // Set the autonomous state
    private void setState(AutoState nextState) {
        autoState = nextState;
        stateStartTime = System.nanoTime();
    }

    // Determine how long we've been in a particular state
    private double secondsInState() {
        return (System.nanoTime() - stateStartTime) / 1e9;
    }


    // ================== PATHS ==============//
    private Path startToScorePath() {
        return line(bottomStartPose, scorePoseBottom)
                .linear(bottomStartPose, scorePoseBottom);
    }
    private Path bottomScoreToCollectPath() {
        return line(scorePoseBottom, collectPose)
                .linear(scorePoseBottom, collectPose);
    }
    private Path collectToTopScorePath() {
        return curve(collectPose, controlCollectToTop, scorePoseTop)
                .linear(collectPose, scorePoseTop);
    }
    private Path topScoreToParkPath() {
        return line(scorePoseTop, upperParkPose)
                .linear(scorePoseTop, upperParkPose);
    }

    // One path for this entire autonomous routine; use if this is a confirmed routine.
    private Path TwoTipAuto() {
        return path(startToScorePath(), bottomScoreToCollectPath(),
                collectToTopScorePath(), topScoreToParkPath());
    }



    public void init() {
        // Create follower and set initial pose
        follower = Constants.create(hardwareMap);
        follower.setPose(bottomStartPose);

        // Create camera-odometry corrector
        cameraCorrector = new CameraOdometryCorrector();
    }

    public void start() {
        setState(AutoState.DRIVE_TO_BOTTOM_SCORE);
        follower.follow(startToScorePath());
    }

    public void loop() {
        follower.update();

        // Compare camera measurements with current Pinpoint localization.
        // Currently diagnostic only; does not modify Pedro's pose.
        cameraCorrector.update(follower.pose());

        switch (autoState) {

            case DRIVE_TO_BOTTOM_SCORE:
                if (!follower.isBusy()) {
                    setState(AutoState.SCORE_BOTTOM);
                }
                break;

            case SCORE_BOTTOM:
                // Future turret and launcher action
                telemetry.addLine("Placeholder: Score bottom Hive");
                follower.follow(bottomScoreToCollectPath());
                setState(AutoState.DRIVE_TO_COLLECT);
                break;

            case DRIVE_TO_COLLECT:
                if (!follower.isBusy()) {
                    setState(AutoState.COLLECT);
                }
                break;

            case COLLECT:
                // Future intake action
                telemetry.addLine("Placeholder: Collect Pollen");
                follower.follow(collectToTopScorePath());
                setState(AutoState.DRIVE_TO_TOP_SCORE);
                break;

            case DRIVE_TO_TOP_SCORE:
                if (!follower.isBusy()) {
                    setState(AutoState.SCORE_TOP);
                }
                break;

            case SCORE_TOP:
                // Future turret and launcher action
                telemetry.addLine("Placeholder: Score top Hive");
                follower.follow(topScoreToParkPath());
                setState(AutoState.DRIVE_TO_PARK);
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    setState(AutoState.FINISHED);
                }
                break;

            case FINISHED:
                break;
        }
    }
}

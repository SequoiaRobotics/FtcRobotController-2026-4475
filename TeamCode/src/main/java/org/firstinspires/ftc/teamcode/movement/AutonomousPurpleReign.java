package org.firstinspires.ftc.teamcode.movement;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.*;

// Subsystems
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TurretStationary;

// Localization
import android.util.Size;
import org.firstinspires.ftc.teamcode.localization.CameraSession;
import org.firstinspires.ftc.teamcode.localization.CameraOdometryCorrector;
import org.firstinspires.ftc.teamcode.localization.HiveOrientationDetector;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class AutonomousPurpleReign extends OpMode {

    // Path follower
    private Follower follower;

    // Subsystems
    private Intake intake;
    private TurretStationary turretStationary;

    // Localization
    private CameraOdometryCorrector cameraCorrector;
    private HiveOrientationDetector hiveOrientationDetectorCam1;
    private HiveOrientationDetector hiveOrientationDetectorCam2;
    private CameraSession camera1;
    private CameraSession camera2;
    private static final boolean USE_VISION = false;
    private static final boolean USE_SECOND_CAMERA = false;

    // CAMERA 1 PLACEHOLDER OFFSETS

    private static final double CAM1_RIGHT = 6.0;
    private static final double CAM1_FORWARD = 6.5;
    private static final double CAM1_UP = 13.5;

    private static final double CAM1_YAW = 0.0;
    private static final double CAM1_PITCH = -90.0;
    private static final double CAM1_ROLL = 0.0;


    // CAMERA 2 PLACEHOLDER OFFSETS

    private static final double CAM2_RIGHT = 0.0;
    private static final double CAM2_FORWARD = 0.0;
    private static final double CAM2_UP = 0.0;

    private static final double CAM2_YAW = 0.0;
    private static final double CAM2_PITCH = -90.0;
    private static final double CAM2_ROLL = 0.0;


    // Pose factory
    private final PoseFactory p = PoseFactory.degrees();


    //============== EDITABLE FIELD POSITIONS ============//

    // Edit based on where the robot starts on the field
    private final Pose bottomStartPoseRed   = p.of(58, 7.75, 90);
    private final Pose topStartPoseRed = p.of(58, 136.25, 270);
    private final Pose parkStartPoseRed = p.of(7.75, 72, 0);
    private final Pose parkPoseRedGap = p.of(14, 72, 0);
    private final Pose topStartPoseBlue = p.of(86, 136.25, 270);
    private final Pose bottomStartPoseBlue = p.of(86, 7.75, 90);
    private final Pose parkStartPoseBlue = p.of(136.25, 72, 180);
    private final Pose parkPoseBlueGap = p.of(129, 72, 180);

    // Edit based on how close to the hive the turret must be, and the orientation of the turret.
    private final Pose scorePoseBottomRed   = p.of(58, 30, 90);
    private final Pose scorePoseTopRed = p.of(58, 110, 270);
    private final Pose scorePoseBottomBlue = p.of(86, 30, 90);
    private final Pose scorePoseTopBlue = p.of(86, 110, 270);

    // Edit based on needed collect position.
    private final Pose lowerRedCollectPose = p.of(12, 48, 180);
    private final Pose upperRedCollectPose = p.of(48, 134, 90);
    private final Pose lowerBlueCollectPose = p.of(96, 12, 270);
    private final Pose upperBlueCollectPose = p.of(134,96, 0);

    // Placeholder control point for collect --> score. Remove / edit if needed
    private final Pose controlCollectToTopRed = p.of(34, 115, 0);
    private final Pose controlScoreToParkRed = p.of(20, 30, 0);
    private final Pose controlCollectToBottomBlue = p.of(110, 29, 0);
    private final Pose controlScoreToParkBlue = p.of(126, 114, 0);

    // Edit based on final parking position
    private final Pose upperParkPoseRed = p.of(10, 118, 0);
    private final Pose lowerParkPoseRed = p.of(10, 98, 0);
    private final Pose lowerParkPoseBlue = p.of(134, 26, 180);
    private final Pose upperParkPoseBlue = p.of(134, 46, 180);


    // Selected alliance / strategy
    private Alliance selectedAlliance = Alliance.RED;
    private AutoStrategy selectedStrategy = AutoStrategy.ONE_TIP;

    // Tracks previous button states for one selection per press.
    private boolean previousLeft = false;
    private boolean previousRight = false;
    private boolean previousUp = false;
    private boolean previousDown = false;


    // State machines for autonomous
    private enum AutoState {
        DRIVE_TO_FIRST_SCORE,
        SCORE_FIRST,
        DRIVE_TO_COLLECT,
        COLLECT,
        DRIVE_TO_SECOND_SCORE,
        SCORE_SECOND,
        DRIVE_TO_PARK,
        FINISHED
    }

    private enum Alliance {
        RED,
        BLUE
    }

    private enum AutoStrategy {
        TWO_TIP,
        ONE_TIP,
        PARK_ONLY,
        VISION_PRELOAD
    }

    private Pose selectedStartPose() {

        if (selectedStrategy == AutoStrategy.PARK_ONLY) {
            return selectedAlliance == Alliance.RED
                    ? parkStartPoseRed
                    : parkStartPoseBlue;
        }

        return selectedAlliance == Alliance.RED
                ? bottomStartPoseRed
                : topStartPoseBlue;
    }

    private AutoState autoState = AutoState.DRIVE_TO_FIRST_SCORE;
    private long stateStartTime;

    // Maximum time to wait for flywheel readiness after arriving.
    private static final double FLYWHEEL_READY_TIMEOUT = 4.0;

    // -1 means the waiting timer hasn't started.
    private long flywheelWaitStart = -1;

    // Set the autonomous state
    private void setState(AutoState nextState) {
        autoState = nextState;
        stateStartTime = System.nanoTime();
    }

    // Determine how long we've been in a particular state
    private double secondsInState() {
        return (System.nanoTime() - stateStartTime) / 1e9;
    }


    // ================== RED ALLIANCE PATHS ==============//
    private Path redStartToScorePathBottom() {
        return line(bottomStartPoseRed, scorePoseBottomRed)
                .linear(bottomStartPoseRed,scorePoseBottomRed);
    }
    private Path redStartToScorePathTop() {
        return line(topStartPoseRed, scorePoseTopRed)
                .linear(topStartPoseRed, scorePoseTopRed);
    }
    private Path redBottomScoreToCollectPath() {
        return line(scorePoseBottomRed, lowerRedCollectPose)
                .linear(scorePoseBottomRed, lowerRedCollectPose);
    }
    private Path redBottomScoreToParkPath() {
        return curve(scorePoseBottomRed, controlScoreToParkRed, upperParkPoseRed)
                .linear(scorePoseBottomRed, upperParkPoseRed);
    }
    private Path redCollectToTopScorePath() {
        return curve(lowerRedCollectPose, controlCollectToTopRed, scorePoseTopRed)
                .linear(lowerRedCollectPose, scorePoseTopRed);
    }
    private Path redTopScoreToParkPath() {
        return line(scorePoseTopRed, upperParkPoseRed)
                .linear(scorePoseTopRed, upperParkPoseRed);
    }

    private Path redStartToParkPath() {
        return path(line(parkStartPoseRed, parkPoseRedGap)
                .linear(parkStartPoseRed, parkPoseRedGap),
        line(parkPoseRedGap, lowerParkPoseRed)
                .linear(parkPoseRedGap, lowerParkPoseRed));
    }


    // ================== PATHS FOR BLUE ALLIANCE ===============//
    private Path blueStartToScorePathTop() {
        return line(topStartPoseBlue, scorePoseTopBlue)
                .linear(topStartPoseBlue, scorePoseTopBlue);
    }
    private Path blueStartToScorePathBottom() {
        return line(bottomStartPoseBlue, scorePoseBottomBlue)
                .linear(bottomStartPoseBlue, scorePoseBottomBlue);
    }
    private Path blueTopScoreToCollectPath() {
        return line(scorePoseTopBlue, upperBlueCollectPose)
                .linear(scorePoseTopBlue, upperBlueCollectPose);
    }
    private Path blueTopScoreToParkPath() {
        return curve(scorePoseTopBlue, controlScoreToParkBlue, lowerParkPoseBlue)
                .linear(scorePoseTopBlue, lowerParkPoseBlue);
    }
    private Path blueCollectToBottomScorePath() {
        return curve(upperBlueCollectPose, controlCollectToBottomBlue, scorePoseBottomBlue)
                .linear(upperBlueCollectPose, scorePoseBottomBlue);
    }
    private Path blueBottomScoreToParkPath() {
        return line(scorePoseBottomBlue, lowerParkPoseBlue)
                .linear(scorePoseBottomBlue, lowerParkPoseBlue);
    }
    private Path blueStartToParkPath() {
        return path(line(parkStartPoseBlue, parkPoseBlueGap)
                        .linear(parkStartPoseBlue, parkPoseBlueGap),
        line(parkPoseBlueGap, upperParkPoseBlue)
                .linear(parkPoseBlueGap, upperParkPoseBlue));
    }


    // ============== SELECTED ROUTES ============== //

    // The initial scoring path for the selected alliance.
    private Path firstScorePath() {
        if (selectedStrategy == AutoStrategy.VISION_PRELOAD) {
            return selectedAlliance == Alliance.RED
                    ? redStartToScorePathTop()
                    : blueStartToScorePathBottom();
        }
        else {
            return selectedAlliance == Alliance.RED
                    ? redStartToScorePathBottom()
                    : blueStartToScorePathTop();
        }

    }

    // Drive from first scoring position to the Flower.
    private Path scoreToCollectPath() {
        return selectedAlliance == Alliance.RED
                ? redBottomScoreToCollectPath()
                : blueTopScoreToCollectPath();
    }

    // Drive from the Flower to the second scoring position.
    private Path collectToSecondScorePath() {
        return selectedAlliance == Alliance.RED
                ? redCollectToTopScorePath()
                : blueCollectToBottomScorePath();
    }

    // After the second scoring action, park.
    private Path secondScoreToParkPath() {
        return selectedAlliance == Alliance.RED
                ? redTopScoreToParkPath()
                : blueBottomScoreToParkPath();
    }

    // For the one-tip strategy, skip collection.
    private Path firstScoreToParkPath() {
        return selectedAlliance == Alliance.RED
                ? redBottomScoreToParkPath()
                : blueTopScoreToParkPath();
    }

    // For the park-only strategy, never approach the Hive.
    private Path directParkPath() {
        return selectedAlliance == Alliance.RED
                ? redStartToParkPath()
                : blueStartToParkPath();
    }



    public void init() {
        // Create follower and set initial pose
        follower = Constants.create(hardwareMap);
        follower.setPose(bottomStartPoseRed);

        // Initialize the turret and intake when ready
        intake = new Intake(hardwareMap);
        turretStationary = new TurretStationary(hardwareMap);

        // Create cameras / orientation detectors
        if (USE_VISION) {
            camera1 = CameraSession.initializeCamera(
                    hardwareMap, "Webcam 1", new Size(640, 480),
                    CAM1_RIGHT, CAM1_FORWARD, CAM1_UP,
                    CAM1_YAW, CAM1_PITCH, CAM1_ROLL
            );

            if (USE_SECOND_CAMERA) {
                camera2 = CameraSession.initializeCamera(
                        hardwareMap, "Webcam 2", new Size(640, 480),
                        CAM2_RIGHT, CAM2_FORWARD, CAM2_UP,
                        CAM2_YAW, CAM2_PITCH, CAM2_ROLL
                );
            }
        }

        // Create camera-odometry corrector
        // cameraCorrector = new CameraOdometryCorrector();
    }


    @Override
    public void init_loop() {

        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        // Choose alliance.
        if (left && !previousLeft) {
            selectedAlliance = Alliance.RED;
        }

        if (right && !previousRight) {
            selectedAlliance = Alliance.BLUE;
        }

        // Cycle between ONE_TIP and PARK_ONLY.
        if ((up && !previousUp) || (down && !previousDown)) {

            if (selectedStrategy == AutoStrategy.ONE_TIP) {
                selectedStrategy = AutoStrategy.PARK_ONLY;
            } else {
                selectedStrategy = AutoStrategy.ONE_TIP;
            }
        }

        // Save button states to detect new presses.
        previousLeft = left;
        previousRight = right;
        previousUp = up;
        previousDown = down;

        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Strategy", selectedStrategy);
        telemetry.addLine("D-pad Left/Right: Alliance");
        telemetry.addLine("D-pad Up/Down: Strategy");
        telemetry.update();
    }



    @Override
    public void start() {

        // Reset localization to the chosen physical starting pose.
         follower.setPose(selectedStartPose());

         if (selectedStrategy == AutoStrategy.PARK_ONLY) {
             follower.follow(directParkPath());
             setState(AutoState.DRIVE_TO_PARK);

         } else {
             // Both scoring strategies begin at the first Hive.
             follower.follow(firstScorePath());
             setState(AutoState.DRIVE_TO_FIRST_SCORE);
         }

         if (USE_VISION) {
             // Only initialize hive orientation detectors after the alliance color is CONFIRMED
             hiveOrientationDetectorCam1 = new HiveOrientationDetector(
                     camera1.processor, selectedAlliance == Alliance.RED);

             if (USE_SECOND_CAMERA) {
                 hiveOrientationDetectorCam2 = new HiveOrientationDetector(
                         camera2.processor, selectedAlliance == Alliance.RED);
             }
         }
    }


    @Override
    public void loop() {

        // ============== UPDATE SYSTEMS ============== //

        follower.update();
        turretStationary.update();

        // Diagnostic only; doesn't modify Pedro's pose.
        // cameraCorrector.update(follower.pose());


        // ============== CAMERA VISION ============== //

        if (USE_VISION) {

            // CAMERA 1
            List<AprilTagDetection> fresh1 =
                    camera1.processor.getFreshDetections();

            if (fresh1 != null) {

                hiveOrientationDetectorCam1.update(fresh1);

                // HOD returns the actual validated detection.
                AprilTagClusterDetection cluster =
                        hiveOrientationDetectorCam1.getStableCluster();

                if (cluster != null && cluster.robotPose != null) {

                    double xFTC = cluster.robotPose.getPosition().x;
                    double yFTC = cluster.robotPose.getPosition().y;

                    double headingFTC = cluster.robotPose
                            .getOrientation()
                            .getYaw(AngleUnit.DEGREES);

                    // Diagnostic only until global Hive geometry is verified.
                    telemetry.addData("Camera X FTC", xFTC);
                    telemetry.addData("Camera Y FTC", yFTC);
                    telemetry.addData("Camera Heading FTC", headingFTC);
                }
            }

            // CAMERA 2 (OPTIONAL)
            if (USE_SECOND_CAMERA && camera2 != null) {

                List<AprilTagDetection> fresh2 =
                        camera2.processor.getFreshDetections();

                if (fresh2 != null) {
                    hiveOrientationDetectorCam2.update(fresh2);
                }
            }
        }


        // ============== AUTONOMOUS STATE MACHINE ============== //

        switch (autoState) {

            case DRIVE_TO_FIRST_SCORE:

                // Start spinning the flywheel while driving.
                if (!turretStationary.isFlywheelRunning()) {
                    turretStationary.startFlyWheel();
                }

                // Only consider shooting once the robot has arrived.
                if (!follower.isBusy()) {

                    // Start waiting when the robot first arrives.
                    if (flywheelWaitStart == -1) {
                        flywheelWaitStart = System.nanoTime();
                    }

                    double waitSeconds =
                            (System.nanoTime() - flywheelWaitStart) / 1e9;

                    // Shoot if ready, OR if the timeout has expired.
                    if (turretStationary.isFlywheelReady()
                            || waitSeconds >= FLYWHEEL_READY_TIMEOUT) {

                        flywheelWaitStart = -1;
                        setState(AutoState.SCORE_FIRST);
                    }
                }

                break;


            case SCORE_FIRST:

                if (secondsInState() < turretStationary.FEEDING_DURATION) {
                    turretStationary.indexerOn();
                } else {

                    turretStationary.stop();

                    if (selectedStrategy == AutoStrategy.TWO_TIP) {
                        follower.follow(scoreToCollectPath());
                        setState(AutoState.DRIVE_TO_COLLECT);
                    } else {
                        follower.follow(firstScoreToParkPath());
                        setState(AutoState.DRIVE_TO_PARK);
                    }
                }
                break;


            case DRIVE_TO_COLLECT:

                if (!follower.isBusy()) {
                    setState(AutoState.COLLECT);
                }
                break;


            case COLLECT:

                if (secondsInState() < intake.COLLECT_DURATION) {
                    intake.start();
                } else {

                    intake.stop();

                    follower.follow(collectToSecondScorePath());
                    setState(AutoState.DRIVE_TO_SECOND_SCORE);
                }
                break;


            case DRIVE_TO_SECOND_SCORE:

                if (!turretStationary.isFlywheelRunning()) {
                    turretStationary.startFlyWheel();
                }

                if (!follower.isBusy() && turretStationary.isFlywheelReady()) {
                    setState(AutoState.SCORE_SECOND);
                }
                break;


            case SCORE_SECOND:

                if (secondsInState() < turretStationary.FEEDING_DURATION) {
                    turretStationary.indexerOn();
                } else {

                    turretStationary.stop();

                    follower.follow(secondScoreToParkPath());
                    setState(AutoState.DRIVE_TO_PARK);
                }
                break;


            case DRIVE_TO_PARK:

                if (!follower.isBusy()) {
                    setState(AutoState.FINISHED);
                }
                break;


            case FINISHED:
                break;
        }


        // ============== TELEMETRY ============== //

        telemetry.addData("Auto State", autoState);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Strategy", selectedStrategy);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());


        telemetry.update();
    }


    public void stop() {
        intake.stop();
        turretStationary.stop();

        if (camera1 != null) {
            camera1.portal.close();
        }

        if (camera2 != null) {
            camera2.portal.close();
        }
    }

}

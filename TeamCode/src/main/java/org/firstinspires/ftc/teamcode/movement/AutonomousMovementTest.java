
package org.firstinspires.ftc.teamcode.movement;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.*;

@Autonomous(name = "Autonomous Movement Test")
public class AutonomousMovementTest extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private boolean redAlliance = true;

    // Same coordinates as AutonomousPurpleReign.
    private final Pose redStart = p.of(58, 7.75, 90);
    private final Pose redScore = p.of(58, 30, 90);
    private final Pose redPark = p.of(10, 120, 180);
    private final Pose redControl = p.of(20, 30, 0);

    private final Pose blueStart = p.of(86, 136.25, 270);
    private final Pose blueScore = p.of(86, 110, 270);
    private final Pose bluePark = p.of(134, 24, 0);
    private final Pose blueControl = p.of(126, 114, 0);

    private Pose startPose;
    private Pose scorePose;
    private Pose parkPose;
    private Pose controlPose;

    private enum TestState {
        DRIVE_TO_SCORE,
        DRIVE_TO_PARK,
        FINISHED
    }

    private TestState state = TestState.DRIVE_TO_SCORE;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void init_loop() {

        if (gamepad1.dpad_left) {
            redAlliance = true;
        }

        if (gamepad1.dpad_right) {
            redAlliance = false;
        }

        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addLine("D-pad Left: Red | Right: Blue");
        telemetry.update();
    }

    @Override
    public void start() {

        startPose = redAlliance ? redStart : blueStart;
        scorePose = redAlliance ? redScore : blueScore;
        parkPose = redAlliance ? redPark : bluePark;
        controlPose = redAlliance ? redControl : blueControl;

        follower.setPose(startPose);

        follower.follow(
                line(startPose, scorePose)
                        .linear(startPose, scorePose)
        );

        state = TestState.DRIVE_TO_SCORE;
    }

    @Override
    public void loop() {

        follower.update();

        switch (state) {

            case DRIVE_TO_SCORE:
                if (!follower.isBusy()) {

                    follower.follow(
                            curve(scorePose, controlPose, parkPose)
                                    .linear(scorePose, parkPose)
                    );

                    state = TestState.DRIVE_TO_PARK;
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    state = TestState.FINISHED;
                }
                break;

            case FINISHED:
                break;
        }

        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("State", state);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)",
                Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}

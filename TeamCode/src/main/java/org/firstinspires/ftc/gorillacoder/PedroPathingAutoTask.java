package org.firstinspires.ftc.gorillacoder;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedro.GorillaDriveConstants;

public class PedroPathingAutoTask<OpModeT extends AbstractOpMode<OpModeT>> extends DriveTankTask<OpModeT> {
    public PedroPathingAutoTask(Follower follower, double[] poses) {
        this.follower = follower;
        this.poses    = poses;
    }

    @Override
    public PedroPathingAutoTask<OpModeT> run() {
        PoseFactory factory;

        super.run();

        factory = PoseFactory.degrees();
        for (int poseIndex = 0; poseIndex < 3 * poses.length; poseIndex += 3) {
            Pose pose = factory.of(poses[poseIndex*3 + 0], poses[poseIndex*3 + 1], poses[poseIndex*3 + 2]);
        }

        return this;
    }

    @Override
    public PedroPathingAutoTask<OpModeT> init() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "PedroPathingAutoTask init: start");

        super.init();

        this.frequencyMillis(50); // adjust motors every 10ms, 100/second
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "PedroPathingAutoTask init: done");
        return this;
    }

    protected Follower follower;
    protected double[] poses;

} // class PedroPathingAutoTask

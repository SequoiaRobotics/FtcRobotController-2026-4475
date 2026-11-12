package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GorillaDriveConstants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
        config -> {
            config.frontLeftName.set("Drive Front Left");
            config.backLeftName.set("Drive Rear Left");
            config.frontRightName.set("Drive Front Right");
            config.backRightName.set("Drive Rear Right");

            config.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            config.manualBrakeMode.set(true);
        }
    );

    public static OTOSConfig otosConfig = new OTOSConfig(
        config -> {
            config.name.set("otos");
            config.linearUnit.set(INCH);
            config.offset.set(new Pose(0, 0)); // x, y offset from center of bot.
            config.angularScalar.set(1.0);
            config.linearScalar.set(1.0);
        }
    );

    public static PinpointConfig pingpointConfig = new PinpointConfig(
        config -> {
            config.name.set("pinpoint");
            config.xPodOffset.set(2.187);
            config.yPodOffset.set(-4.572);
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
        config -> {
            Controller primaryTranslationalForward = Controller.proportional(0.3);
            Controller secondaryTranslationalForward = Controller.proportional(0.1);
            Controller primaryTranslationalLateral = Controller.proportional(0.3);
            Controller secondaryTranslationalLateral = Controller.proportional(0.1);

            config.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            config.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            config.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
            config.brake.set(Controller.proportionalFeedforward(0.008731598255925491));

            config.headingFeedback.set(Controller.proportional(5.258721785960744));
            config.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));

            config.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
            config.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

            config.maxAchievableForwardVelocity.set(72.72923108818539);
            config.maxAchievableStrafeVelocity.set(52.34323936525474);
            config.naturalForwardDeceleration.set(85.01144677379789);
            config.naturalStrafeDeceleration.set(104.49787535782846);
        }
    );

    public static Follower create(HardwareMap hardwareMap) {
        OTOSLocalizer otosLocalizer = new OTOSLocalizer(hardwareMap, otosConfig);
        return new Follower(
            new PinpointLocalizer(hardwareMap, pingpointConfig),
            new Mecanum(hardwareMap, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
} // class Constants
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class GorillaGobildaWoodFloorDriveConstants {
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

    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.xPodOffset.set(2.5851539551742433);
        c.yPodOffset.set(0.7005141100545568);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(  0.2743562706583265);
            Controller secondaryTranslationalForward = Controller.proportional(0.10136731107368391);
            Controller primaryTranslationalLateral = Controller.proportional(  0.3792459408802851);
            Controller secondaryTranslationalLateral = Controller.proportional(0.14012124151709116);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put( 2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.017815241471564033));
            c.brake.set(Controller.proportionalFeedforward(0.015142955250829428));

            c.headingFeedback.set(Controller.proportional(   3.2475975872320446));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04371390636724927, 0.013456523701095657));

            c.linearBrakeCoefficients.set(Matrix.diag(    0.021165181104493735, 0.01954096756876752));
            c.quadraticBrakeCoefficients.set(Matrix.diag(-1.33224602965156E-4,  0.00490427652681007));

            c.maxAchievableForwardVelocity.set(57.52897618837416);
            c.maxAchievableStrafeVelocity.set( 49.77290756058593);
            c.naturalForwardDeceleration.set(  26.361821088461316);
            c.naturalStrafeDeceleration.set(   39.000065875624685);
        }
    );
    public static Follower create(HardwareMap hardwareMap) {
        OTOSLocalizer otosLocalizer = new OTOSLocalizer(hardwareMap, otosConfig);
        return new Follower(
            new PinpointLocalizer(hardwareMap, pinpointConfig),
            new Mecanum(hardwareMap, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
} // class GorillaGobildaWoodFloorDriveConstants
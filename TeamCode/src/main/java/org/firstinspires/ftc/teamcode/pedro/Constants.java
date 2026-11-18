package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // Tuned mecanum config
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Tuned localizer config here
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.2146367801456002);
        c.yPodOffset.set(-7.696790319728101);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Tuned foresight config here
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.24301798960948387);
                Controller secondaryTranslationalForward = Controller.proportional(0.08978865360042831);
                Controller primaryTranslationalLateral = Controller.proportional(0.2796066474880521);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10330718501961479);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017394282081932135));
                c.brake.set(Controller.proportionalFeedforward(0.014785139769642314));

                c.headingFeedback.set(Controller.proportional(3.033394115885183));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04854201511389972, 0.003682333312835248));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08290498253031257, 0.06581308922565825));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011471154094721474, 0.0014705619731211028));

                c.maxAchievableForwardVelocity.set(60.3934973642548);
                c.maxAchievableStrafeVelocity.set(50.70266948957136);
                c.naturalForwardDeceleration.set(45.532461739155764);
                c.naturalStrafeDeceleration.set(70.70050907552461);
            }
    );

    // Create follower w hardware map
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}
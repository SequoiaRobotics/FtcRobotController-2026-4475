package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

public class GorillaGobildaWoodFloorTuning {

    // 1. Drivetrain Tuner: Configures motor names and directions
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    // 2. Localization Tuner: Configures encoder directions and pod offsets
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // 3. Foresight Tuner: Automatically tunes velocities, braking, and kP values
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
            (hardwareMap) -> new PinpointLocalizer(hardwareMap, GorillaGobildaWoodFloorDriveConstants.pinpointConfig),
            (hardwareMap) -> new Mecanum(hardwareMap, GorillaGobildaWoodFloorDriveConstants.drivetrainConfig)
        );
    }
}

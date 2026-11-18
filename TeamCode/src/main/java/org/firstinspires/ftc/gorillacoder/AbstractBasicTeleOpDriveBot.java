package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@SuppressWarnings("unused")
public abstract class AbstractBasicTeleOpDriveBot extends AbstractBasicDriveBot {

    public abstract static class TeleOpDriveTask extends AbstractBotTask<AbstractBasicDriveBot> {
    }

    @Override
    protected AbstractBasicDriveBot configureBot() {
        return super.configureBot();
    }

    protected abstract void readGamePad();

    protected class GamePadTask extends TeleOpDriveTask {
        @Override
        public GamePadTask run() {
            DriveMode priorDriveMode = opMode.driveMode;
            readGamePad();
            if (priorDriveMode != opMode.driveMode) {
                disable(driveTasks.get(priorDriveMode));
                enable(driveTasks.get(opMode.driveMode));
            }

            return this;
        }

        @Override
        public GamePadTask init() {
            frequencyMillis(5); // Every 5 ms, 200/second.
            return this;
        }
    } // class GamePadTask

    @Override
    protected List<BotTask<AbstractBasicDriveBot>> getTasks() {
        @SuppressWarnings("UnnecessaryLocalVariable")
        List<BotTask<AbstractBasicDriveBot>> result = new ArrayList<BotTask<AbstractBasicDriveBot>>( List.of(
            gamePadTask,
            visionTask,
            drivePovTask,
            new TelemetryTask()
        ));

        return result;
    }

    protected GamePadTask gamePadTask   = new GamePadTask();

} // abstract class AbstractBasicTeleOpDriveBot

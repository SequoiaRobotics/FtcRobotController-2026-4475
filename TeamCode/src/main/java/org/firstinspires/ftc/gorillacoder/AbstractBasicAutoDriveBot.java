package org.firstinspires.ftc.gorillacoder;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
public abstract class AbstractBasicAutoDriveBot extends AbstractBasicDriveBot {

    public abstract static class AutoDriveTask extends AbstractBotTask<AbstractBasicAutoDriveBot> {
    }

    protected AbstractBasicAutoDriveBot configureBot() {
        super.configureBot();

        return this;
    }

    @Override
    protected List<BotTask<AbstractBasicDriveBot>> getTasks() {
        @SuppressWarnings("unchecked")
        List<BotTask<AbstractBasicDriveBot>> result = new ArrayList( List.of(
            visionTask,
            bulkReadTask,
            drivePovTask, // initially POV drive mode. Other drive modes may be switched to later.
            new TelemetryTask()
        ));



        return result;
    }

} // abstract class AbstractBasicAutoDriveBot

package org.firstinspires.ftc.gorillacoder;

@SuppressWarnings("unused")
public abstract class AbstractBasicAutoDriveBot extends AbstractBasicDriveBot {

    public abstract static class AutoDriveTask extends AbstractBotTask<AbstractBasicAutoDriveBot> {
    }

    protected AbstractBasicAutoDriveBot configureBot() {
        super.configureBot();

        return this;
    }

    @Override
    protected BotTask<AbstractBasicDriveBot>[] getTasks() {
        @SuppressWarnings("unchecked")
        BotTask<AbstractBasicDriveBot>[] result = new BotTask[] {
            visionTask,
            drivePovTask,
            new TelemetryTask()
        };



        return result;
    }

} // abstract class AbstractBasicAutoDriveBot

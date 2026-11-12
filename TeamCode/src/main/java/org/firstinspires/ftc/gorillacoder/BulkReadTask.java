package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

// BulkReadTask reads encoders (and other hub sensor pins) in bulk.
@SuppressWarnings("unused")
public class BulkReadTask<OpModeT extends AbstractOpMode<OpModeT>> extends AbstractDriveTask<OpModeT>  {

    @Override
    public BulkReadTask<OpModeT> run() {
        // TODO: We don't really know when this runs relative to other tasks.
        // Does it run before or after drive tasks, or tasks that read.
        // Need to figure out when to run relative to each of the above
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() start", getClass().getSimpleName());
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() done", getClass().getSimpleName());
        return this;
    }

    @Override
    public BulkReadTask<OpModeT> init() {
        super.init();
        this.frequencyMillis(100); // reset every 100ms, 10/second. Is this too often?

        allHubs = this.opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        return this;
    }

    // TODO: Move to task
    protected List<LynxModule> allHubs;

} // class BulkReadTask



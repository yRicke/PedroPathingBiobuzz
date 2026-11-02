package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

@Autonomous
public class ExampleAuto extends OpMode {

    private Follower follower; // Add this
    private boolean autonomousStarted;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(8.4178, 29.1432, 270);
    private final Pose point2 = poseFactory.of(8.4185, 8.3935, 270);
    private final Pose point3 = poseFactory.of(57.3377, 127.1916, 270);
    private final Pose point3Control1 = poseFactory.of(33.9725, 29.7519, 0);
    private final Pose point3Control2 = poseFactory.of(16.094, 117.2188, 0);
    private final Pose point4 = poseFactory.of(2.3025, 105.9985, 270);

    @Override
    public void init() {
        OpModeStorage.clearAutonomousPose();
        autonomousStarted = false;
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
    }

    @Override
    public void start() {
        autonomousStarted = true;
    }

    @Override
    public void loop() {
        follower.update();
    }

    @Override
    public void stop() {
        try {
            if (autonomousStarted) {
                follower.localizer.update();
                OpModeStorage.saveAutonomousPose(follower.pose());
            }
        } finally {
            if (follower != null) {
                follower.stop();
                follower.drivetrain.stop();
            }
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }
}

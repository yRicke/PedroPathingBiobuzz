package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name = "AutoPath", group = "Autonomous")

public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(34.707, 133.4141, -90);
    private final Pose point1 = poseFactory.of(11.5827, 117.8106, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {
        OpModeStorage.clearAutonomousPose();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        boolean autonomousStarted = false;
        try {
            follower.setPose(start);
            follower.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }
            autonomousStarted = true;
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
                idle();
            }
        } finally {
            try {
                if (autonomousStarted) {
                    follower.localizer.update();
                    OpModeStorage.saveAutonomousPose(follower.pose());
                }
            } finally {
                try {
                    Scheduler.reset();
                } finally {
                    follower.stop();
                    follower.drivetrain.stop();
                }
            }
        }
    }

    public Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }
}


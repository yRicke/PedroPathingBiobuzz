package org.firstinspires.ftc.teamcode.autos;


import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import static com.pedropathing.ivy.Scheduler.schedule;

@Autonomous(name = "AutonomousSouthOutside", group = "Autonomous")
public class AutonomousSouthOutside extends LinearOpMode {

    private Follower follower;
    private Intake intake;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9, 90);
    private final Pose foward = poseFactory.of(56,15,90);
    private final Pose path1 = poseFactory.of(31, 15, 180);
    private final Pose point2 = poseFactory.of(5, 15, 180);
    private final Pose point3Control1 = poseFactory.of(79, 1, 0);
    private final Pose point4 = poseFactory.of(4, 116, 0.0923);
    private final Pose point4Control1 = poseFactory.of(57, 116, 0);

    DcMotor intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");

    Command LigaIntake = Command.build()
            .setExecute(() -> intakeMotor.setPower(1));

    Command DesligaIntake = Command.build()
            .setExecute(() -> intakeMotor.setPower(0));



    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower,foward()),
                LigaIntake,
                waitMs(3000),
                DesligaIntake,
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }


    @Override
    public void runOpMode() {
        Scheduler.reset();




        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();






        waitForStart();
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
        }
    }



    public Path foward() {
        return Paths.line(start, foward).linear(start, foward);

    }
    public Path path1() {
        return Paths.line(foward, path1).linear(foward, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }
}

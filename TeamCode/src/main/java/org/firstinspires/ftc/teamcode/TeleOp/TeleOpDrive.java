package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.FlowerCollect;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

/** TeleOp unico com localizacao do Pedro e ajuste final de mira pela Limelight. */
@TeleOp(name = "TeleOpDrive", group = "TeleOpMode")
public class TeleOpDrive extends LinearOpMode {
    // Referencial adotado para a alianca atual: X/Y em polegadas, heading em radianos.
    private static final Pose INITIAL_POSE = new Pose(8.4, 35.2, Math.toRadians(0));
    private static final Pose RESET_POSE = new Pose(8.4, 35.2, Math.toRadians(0));


    private Drive drive;
    private Limelight limelight;
    private Intake intake;
    private Flywheel flywheel;
    private Transfer transfer;
    private Trigger trigger;
    private FlowerCollect flowerCollect;
//<<<<<<< HEAD

//=======
//>>>>>>> d06ca30eaecb8a46b87eb487276679374d961ce3
    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        drive = new Drive(hardwareMap,telemetry);
        intake = new Intake(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        trigger = new Trigger(hardwareMap, telemetry);
        trigger.setTriggerClosed();
        limelight = new Limelight(hardwareMap, telemetry);
        flowerCollect = new FlowerCollect(hardwareMap,telemetry);



            waitForStart();
            if (isStopRequested()) {
                return;
            }



            while (opModeIsActive()) {


                limelight.run(gamepad2.dpad_left, gamepad2.dpad_right,
                        gamepad2.dpad_up, gamepad2.dpad_down);
                AprilTagObservation observation = limelight.getObservation();
                boolean aimRequested = gamepad1.right_bumper;
                boolean shootRequested = gamepad1.right_trigger > 0;
                
               
                drive.run(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_bumper,0);
                
                intake.run(gamepad1.a, gamepad1.left_bumper);
                flywheel.runByDistance(limelight.getDistanceMeters());
                transfer.run(shootRequested, gamepad1.left_bumper);
                trigger.run(shootRequested);
                flowerCollect.run(gamepad1.b);
                drive.sendTelemetry();
                limelight.sendTelemetry();
                intake.sendTelemetry();
                flywheel.sendTelemetry();
                transfer.sendTelemetry();
                trigger.sendTelemetry();
                flowerCollect.sendTelemetry();
                telemetry.update();
                idle();
            }
        }
            }







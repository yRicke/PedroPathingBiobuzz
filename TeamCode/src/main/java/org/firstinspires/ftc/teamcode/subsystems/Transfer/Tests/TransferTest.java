package org.firstinspires.ftc.teamcode.subsystems.Transfer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;

@TeleOp(name = "Transfer Test", group = "Transfer")
public class TransferTest extends LinearOpMode {

    private Transfer transfer;

    @Override
    public void runOpMode() {
        transfer = new Transfer(hardwareMap, telemetry);

        waitForStart();
        try {
            while (opModeIsActive()) {
                transfer.run(gamepad1.right_trigger > 0, gamepad1.left_bumper);
                transfer.sendTelemetry();
                telemetry.update();
            }
        } finally {
            transfer.offMotor();
        }
    }
}

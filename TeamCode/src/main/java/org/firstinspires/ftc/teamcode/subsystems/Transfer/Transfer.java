package org.firstinspires.ftc.teamcode.subsystems.Transfer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {

    private DcMotorEx transferMotor;
    private double transferMaxPower = 0.7;

    private Telemetry telemetry;

    // Construtor recebe o hardwareMap e telemetry do OpMode.
    public Transfer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        transferMotor.setDirection(DcMotor.Direction.REVERSE);
        offMotor();
    }

    // Mantem o motor ligado somente enquanto o gatilho estiver pressionado.
    public void run(boolean rightTrigger) {
        if (rightTrigger) {
            onMotor();
        } else {
            offMotor();
        }
    }

    public void onMotor() {
        transferMotor.setPower(transferMaxPower);
    }

    public void offMotor() {
        transferMotor.setPower(0);
    }

    public void sendTelemetry() {
        telemetry.addData("Transfer Power", transferMotor.getPower());
    }
}

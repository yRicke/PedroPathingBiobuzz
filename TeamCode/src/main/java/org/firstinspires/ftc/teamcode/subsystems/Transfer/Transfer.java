package org.firstinspires.ftc.teamcode.subsystems.Transfer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {

    private DcMotorEx transferMotor;
    private double transferMaxPower = 1;

    private Telemetry telemetry;

    // Construtor recebe o hardwareMap e telemetry do OpMode.
    public Transfer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        offMotor();
    }

    // Mantem o motor ligado somente enquanto o gatilho estiver pressionado.
    public void run(boolean on) {
        run(on, false);
    }

    public void run(boolean on, boolean eject) {
        if (eject) {
            eject();
        } else if (on) {
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

    public void eject() {
        transferMotor.setPower(-transferMaxPower);
    }

    public void sendTelemetry() {
        telemetry.addData("Transfer Power", transferMotor.getPower());
    }
}

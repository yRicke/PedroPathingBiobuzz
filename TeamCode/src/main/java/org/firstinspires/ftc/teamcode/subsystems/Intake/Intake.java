package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private DcMotorEx intakeMotor = null;
    private double intakeMaxPower = 1;
    private boolean intakeOn = false;
    private boolean previousOn = false;

    private Telemetry telemetry;

    // Construtor recebe o hardwareMap e telemetry do OpMode
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    // On alterna o intake; eject inverte enquanto pressionado.
    public void run(boolean on, boolean eject) {
        if (on && !previousOn) {
            intakeOn = !intakeOn;
        }
        previousOn = on;

        if (eject) {
            eject();
        } else if (intakeOn) {
            onMotor();
        } else {
            offMotor();
        }
    }

    public void onMotor(){
        intakeMotor.setPower(-intakeMaxPower);
    }
    public void offMotor(){
        intakeMotor.setPower(0);
    }
    public void eject(){
        intakeMotor.setPower(intakeMaxPower);
    }


    // Telemetria
    public void sendTelemetry() {
        telemetry.addData("Intake Power", intakeMotor.getPower());
    }
}

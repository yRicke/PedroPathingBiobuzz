package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private DcMotorEx intakeMotorOne = null;
    private DcMotorEx intakeMotorTwo = null;
    private double intakeMaxPower = 1.0;

    private Telemetry telemetry;

    // Construtor recebe o hardwareMap e telemetry do OpMode
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotorOne = hardwareMap.get(DcMotorEx.class, "intake_motor_one");
        intakeMotorOne.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorTwo = hardwareMap.get(DcMotorEx.class, "intake_motor_two");
        intakeMotorTwo.setDirection(DcMotor.Direction.REVERSE);
    }

    // Recolhe continuamente e inverte os motores quando solicitado.
    public void run(boolean eject) {
        if (eject) {
            eject();
        } else {
            onMotor();
        }
    }

    public void onMotor(){
        intakeMotorOne.setPower(intakeMaxPower);
        intakeMotorTwo.setPower(0.7);
    }
    public void offMotor(){
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
    }
    public void eject(){
        intakeMotorOne.setPower(-intakeMaxPower);
        intakeMotorTwo.setPower(-intakeMaxPower);
    }


    // Telemetria
    public void sendTelemetry() {
        telemetry.addData("Intake One Power", intakeMotorOne.getPower());
        telemetry.addData("Intake Two Power", intakeMotorTwo.getPower());
    }
}

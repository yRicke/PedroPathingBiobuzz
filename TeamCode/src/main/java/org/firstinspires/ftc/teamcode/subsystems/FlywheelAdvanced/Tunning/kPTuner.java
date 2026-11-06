package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Tunning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;

@TeleOp(name = "kP Tuner", group = "Flywheel")
public class kPTuner extends OpMode {
    private Flywheel flywheel;

    public static double kV = Flywheel.KV;
    public double kS = Flywheel.KS;
    public double kP = Flywheel.KP;
    public double goalRPM = 2800;

    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    int incrementIdx = 2;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed() && incrementIdx < increments.length - 1) {
            incrementIdx++;
        } else if (gamepad1.dpadRightWasPressed() && incrementIdx > 0) {
            incrementIdx--;
        }

        if (gamepad1.a) {
            goalRPM = 2800;
        }
        else if (gamepad1.b) {
            goalRPM = 2200;
        }

        double currentStep = increments[incrementIdx];

        // change kP
        if (gamepad1.dpadUpWasPressed()) { kP += currentStep; }
        if (gamepad1.dpadDownWasPressed()) { kP = Math.max(0, kP - currentStep); }

        flywheel.setGains(kS, kV, kP);
        flywheel.run(goalRPM);

        telemetry.addData("Step", "%.8f", currentStep);
        telemetry.addData("kP", "%.8f", kP);
        telemetry.addData("Target RPM", goalRPM);
        telemetry.addData("Error", goalRPM - flywheel.getRPM());
        telemetry.addData("RPM", flywheel.getRPM());
    }
    @Override
    public void stop() {
        flywheel.stopMotors();
    }
}

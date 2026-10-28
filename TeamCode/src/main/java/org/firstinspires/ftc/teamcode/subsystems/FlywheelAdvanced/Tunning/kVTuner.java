package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Tunning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.FlywheelConfig;

@TeleOp(name = "kV Tuner", group = "Flywheel")
public class kVTuner extends OpMode {
    private Flywheel flywheel;

    public static double kV = FlywheelConfig.KV;
    public double kS = FlywheelConfig.KS;
    public double goalRPM = 2800;

    double[] increments = {0.00000001,0.0000001,0.000001, 0.00001, 0.0001, 0.001, 0.01};
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

        // change kV
        if (gamepad1.dpadUpWasPressed()) { kV += currentStep; }
        if (gamepad1.dpadDownWasPressed()) { kV = Math.max(0, kV - currentStep); }

        flywheel.setGains(kS, kV, 0);
        flywheel.run(goalRPM);

        telemetry.addData("Step", "%.8f", currentStep);
        telemetry.addData("kV", "%.8f", kV);
        telemetry.addData("Target RPM", goalRPM);
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("Ticks per Sec", flywheel.getTicksPerSec());
    }
    @Override
    public void stop() {
        flywheel.stopMotors();
    }
}

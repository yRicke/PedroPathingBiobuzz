package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Tunning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.FlywheelConfig;

@TeleOp(name = "kS Tuner", group = "Flywheel")
public class kSTuner extends OpMode {
    private Flywheel flywheel;

    public static double kS = FlywheelConfig.KS;

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

        double currentStep = increments[incrementIdx];

        // change kS
        if (gamepad1.dpadUpWasPressed()) { kS += currentStep; }
        if (gamepad1.dpadDownWasPressed()) { kS = Math.max(0, kS - currentStep); }

        flywheel.setMotorPower(kS);

        telemetry.addData("Step", "%.6f", currentStep);
        telemetry.addData("kS", "%.6f", kS);
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("Ticks per Sec", flywheel.getTicksPerSec());
    }
    @Override
    public void stop() {
        flywheel.stopMotors();
    }
}

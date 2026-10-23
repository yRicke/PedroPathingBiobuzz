package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Drive field-centric do Pedro com hold automatico quando os controles sao soltos. */
public class PedroDrive {
    private static final double PRECISION_MULTIPLIER = 0.3;
    private static final double HOLD_INPUT_THRESHOLD = 0.1;
    private static final double HOLD_VELOCITY_THRESHOLD = 0.2;

    private final Follower follower;
    private final Telemetry telemetry;

    public PedroDrive(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
    }

    public void run(double axial, double lateral, double yaw, boolean precisionMode) {
        double multiplier = precisionMode ? PRECISION_MULTIPLIER : 1.0;

        // Pedro usa lateral para a esquerda e giro anti-horario positivos.
        DrivePowers powers = ManualDrive.fieldCentric(
                axial * multiplier,
                -lateral * multiplier,
                -yaw * multiplier,
                follower.pose().heading());

        // Ajusta o limiar junto com a potencia para preservar os comandos no modo precisao.
        ManualDrive.driveOrHold(follower, powers,
                HOLD_INPUT_THRESHOLD * multiplier, HOLD_VELOCITY_THRESHOLD);
        follower.update();
    }

    public void sendTelemetry() {
        telemetry.addData("Drive Mode", follower.mode());
        telemetry.addData("Robot X (in)", follower.pose().x());
        telemetry.addData("Robot Y (in)", follower.pose().y());
        telemetry.addData("Field Heading (deg)", Math.toDegrees(follower.pose().heading()));
    }

    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
    }
}

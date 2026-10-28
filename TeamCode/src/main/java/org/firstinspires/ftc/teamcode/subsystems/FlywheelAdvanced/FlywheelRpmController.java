package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Converte distancia em setpoint estabilizado; o controle dos motores roda a cada ciclo. */
public final class FlywheelRpmController {
    private final FlywheelRpmTable table;
    private final Telemetry telemetry;
    private final LongSupplier nanoClock;
    private final long updateNanos = TimeUnit.MILLISECONDS.toNanos(FlywheelConfig.SETPOINT_UPDATE_MS);
    private double requestedRpm;
    private double targetRpm;
    private long lastUpdateNanos;
    private boolean initialized;
    private FlywheelRpmTable.Mode mode = FlywheelRpmTable.Mode.STOPPED;

    public FlywheelRpmController(Telemetry telemetry) {
        this(FlywheelConfig.createRpmTable(), telemetry, System::nanoTime);
    }

    FlywheelRpmController(FlywheelRpmTable table, Telemetry telemetry, LongSupplier nanoClock) {
        if (table == null || nanoClock == null || updateNanos < 0
                || !Double.isFinite(FlywheelConfig.SETPOINT_DEADBAND_RPM)
                || FlywheelConfig.SETPOINT_DEADBAND_RPM < 0) {
            throw new IllegalArgumentException("Configuracao do setpoint invalida");
        }
        this.table = table;
        this.telemetry = telemetry;
        this.nanoClock = nanoClock;
    }

    public double run(OptionalDouble distanceMeters) {
        long now = nanoClock.getAsLong();
        FlywheelRpmTable.Mode nextMode = table.getMode(distanceMeters);
        requestedRpm = table.calculate(distanceMeters);
        boolean lostDistance = nextMode == FlywheelRpmTable.Mode.NO_DISTANCE && mode != nextMode;
        if (!initialized || requestedRpm == 0 || lostDistance
                || (now - lastUpdateNanos >= updateNanos
                && Math.abs(requestedRpm - targetRpm) >= FlywheelConfig.SETPOINT_DEADBAND_RPM)) {
            targetRpm = requestedRpm;
            lastUpdateNanos = now;
            initialized = true;
        }
        mode = nextMode;
        return targetRpm;
    }

    public void sendTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Pares RPM calibrados", table.getPointCount());
        telemetry.addData("RPM calculada", requestedRpm);
        telemetry.addData("Modo RPM", mode.getDescription());
    }

    public void reset() {
        requestedRpm = targetRpm = 0;
        initialized = false;
        mode = FlywheelRpmTable.Mode.STOPPED;
    }
}

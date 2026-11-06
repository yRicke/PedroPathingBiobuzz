package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Controle de velocidade unidirecional por feedforward + proporcional. */
public class Flywheel {
    public static final String MOTOR_ONE_NAME = "launcher_motor_one";
    public static final String MOTOR_TWO_NAME = "launcher_motor_two";
    public static final double TICKS_PER_REVOLUTION = 28;
    public static final double GEAR_RATIO = 1;
    public static final double KS = 0.078;
    public static final double KV = 0.00015849;
    public static final double KP = 0.00169;
    public static final double BASE_RPM = 2570;
    // Referencia: saida das bolas -> abertura, medida no Flywheel Calibration - 3D.
    // Distancia zero desativa o par correspondente.
    public static final double BASE_DISTANCE_CM = 185;
    public static final double BASE_RPM_2 = 2560;
    public static final double BASE_DISTANCE_2_CM = 182;
    public static final double BASE_RPM_3 = 2680;
    public static final double BASE_DISTANCE_3_CM = 205;
    public static final double MIN_RPM = Math.min(BASE_RPM, Math.min(BASE_RPM_2, BASE_RPM_3));
    public static final double MAX_RPM = Math.max(BASE_RPM, Math.max(BASE_RPM_2, BASE_RPM_3));
    public static final long SETPOINT_UPDATE_MS = 100;
    public static final double SETPOINT_DEADBAND_RPM = 10;

    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private final Telemetry telemetry;
    private double targetRpm;
    private final FlywheelRpmTable rpmTable;
    private final LongSupplier nanoClock;
    private double requestedAutoRpm;
    private double stableAutoRpm;
    private long lastAutoUpdateNanos;
    private boolean autoInitialized;
    private boolean automatic;
    private FlywheelRpmTable.Mode automaticMode = FlywheelRpmTable.Mode.STOPPED;
    private double kS = KS;
    private double kV = KV;
    private double kP = KP;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap.get(DcMotorEx.class, MOTOR_ONE_NAME),
                hardwareMap.get(DcMotorEx.class, MOTOR_TWO_NAME), telemetry);
    }

    Flywheel(DcMotorEx motorOne, DcMotorEx motorTwo, Telemetry telemetry) {
        this(motorOne, motorTwo, telemetry, createRpmTable(), System::nanoTime);
    }

    Flywheel(DcMotorEx motorOne, DcMotorEx motorTwo, Telemetry telemetry,
             FlywheelRpmTable rpmTable, LongSupplier nanoClock) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.telemetry = telemetry;
        if (!Double.isFinite(TICKS_PER_REVOLUTION)
                || TICKS_PER_REVOLUTION <= 0
                || !Double.isFinite(GEAR_RATIO) || GEAR_RATIO <= 0
                || !Double.isFinite(MIN_RPM) || MIN_RPM <= 0
                || !Double.isFinite(MAX_RPM) || MAX_RPM < MIN_RPM) {
            stopMotors();
            throw new IllegalArgumentException("Configuracao mecanica da flywheel invalida");
        }
        if (rpmTable == null || nanoClock == null || SETPOINT_UPDATE_MS < 0
                || !Double.isFinite(SETPOINT_DEADBAND_RPM) || SETPOINT_DEADBAND_RPM < 0) {
            stopMotors();
            throw new IllegalArgumentException("Configuracao do setpoint invalida");
        }
        this.rpmTable = rpmTable;
        this.nanoClock = nanoClock;
        setGains(KS, KV, KP);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        stopMotors();
    }

    /** Potencia no sentido de lancamento; zero acima da velocidade, sem reversao ativa. */
    public void setMotorPower(double power) {
        double bounded = Double.isFinite(power) ? Math.max(0, Math.min(1, power)) : 0;
        motorOne.setPower(bounded);
        motorTwo.setPower(-bounded);
    }

    public void stopMotors() {
        resetAutomaticSetpoint();
        stopMotorOutput();
    }

    private void stopMotorOutput() {
        targetRpm = 0;
        setMotorPower(0);
    }

    public void setGains(double kS, double kV, double kP) {
        if (!Double.isFinite(kS) || !Double.isFinite(kV) || !Double.isFinite(kP)
                || kS < 0 || kV < 0 || kP < 0) {
            stopMotors();
            throw new IllegalArgumentException("Ganhos da flywheel invalidos");
        }
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
    }

    /** Chame a cada ciclo; setpoint invalido ou encoder invalido desliga ambos os motores. */
    public void run(double rpm) {
        resetAutomaticSetpoint();
        applyRpm(rpm);
    }

    /** RPM automatica pela distancia: consulta a tabela e estabiliza o setpoint. */
    public void runByDistance(OptionalDouble distanceMeters) {
        long now = nanoClock.getAsLong();
        FlywheelRpmTable.Mode nextMode = rpmTable.getMode(distanceMeters);
        requestedAutoRpm = boundRpm(rpmTable.calculate(distanceMeters));
        boolean lostDistance = nextMode == FlywheelRpmTable.Mode.NO_DISTANCE && automaticMode != nextMode;
        if (!autoInitialized || requestedAutoRpm == 0 || lostDistance
                || (now - lastAutoUpdateNanos >= TimeUnit.MILLISECONDS.toNanos(SETPOINT_UPDATE_MS)
                && Math.abs(requestedAutoRpm - stableAutoRpm) >= SETPOINT_DEADBAND_RPM)) {
            stableAutoRpm = requestedAutoRpm;
            lastAutoUpdateNanos = now;
            autoInitialized = true;
        }
        automaticMode = nextMode;
        automatic = true;
        applyRpm(stableAutoRpm);
    }

    private void resetAutomaticSetpoint() {
        requestedAutoRpm = stableAutoRpm = 0;
        autoInitialized = automatic = false;
        automaticMode = FlywheelRpmTable.Mode.STOPPED;
    }

    private void applyRpm(double rpm) {
        double currentRpm = getRPM();
        if (!Double.isFinite(rpm) || rpm <= 0 || !Double.isFinite(currentRpm)) {
            stopMotorOutput();
            return;
        }
        targetRpm = boundRpm(rpm);
        double power = kS + kV * targetRpm + kP * (targetRpm - currentRpm);
        setMotorPower(power);
    }

    private static double boundRpm(double rpm) {
        return Double.isFinite(rpm) && rpm > 0 ? Math.max(MIN_RPM, Math.min(rpm, MAX_RPM)) : 0;
    }

    /** Pares {distancia 3D da saida ate a abertura em cm, RPM}; zero desativa o par. */
    private static FlywheelRpmTable createRpmTable() {
        return new FlywheelRpmTable(BASE_RPM, MAX_RPM, new double[][] {
                {BASE_DISTANCE_CM, BASE_RPM},
                {BASE_DISTANCE_2_CM, BASE_RPM_2},
                {BASE_DISTANCE_3_CM, BASE_RPM_3}
        });
    }

    public double getTargetRPM() { return targetRpm; }
    public double getTicksPerSec() { return motorOne.getVelocity(); }
    public double getRPM() { return toRpm(getTicksPerSec()); }
    public double getSecondMotorRPM() { return toRpm(-motorTwo.getVelocity()); }

    private double toRpm(double ticksPerSecond) {
        return ticksPerSecond * 60 / TICKS_PER_REVOLUTION / GEAR_RATIO;
    }

    public void sendTelemetry() {
        if (telemetry == null) return;
        double currentRpm = getRPM();
        telemetry.addData("Flywheel RPM alvo", targetRpm);
        telemetry.addData("Flywheel limites RPM", "%.0f - %.0f", MIN_RPM, MAX_RPM);
        telemetry.addData("Flywheel RPM medido", currentRpm);
        telemetry.addData("Flywheel RPM motor 2", getSecondMotorRPM());
        telemetry.addData("Flywheel erro RPM", targetRpm - currentRpm);
        telemetry.addData("Flywheel potencia", motorOne.getPower());
        if (automatic) {
            telemetry.addData("Pares RPM calibrados", rpmTable.getPointCount());
            telemetry.addData("RPM calculada", requestedAutoRpm);
            telemetry.addData("Modo RPM", automaticMode.getDescription());
        } else {
            telemetry.addData("Modo RPM", "Manual");
        }
    }
}

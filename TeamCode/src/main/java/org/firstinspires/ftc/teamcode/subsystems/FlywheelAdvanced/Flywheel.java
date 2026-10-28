package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Controle de velocidade unidirecional por feedforward + proporcional. */
public class Flywheel {
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private final Telemetry telemetry;
    private double targetRpm;
    private double kS = FlywheelConfig.KS;
    private double kV = FlywheelConfig.KV;
    private double kP = FlywheelConfig.KP;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap.get(DcMotorEx.class, FlywheelConfig.MOTOR_ONE_NAME),
                hardwareMap.get(DcMotorEx.class, FlywheelConfig.MOTOR_TWO_NAME), telemetry);
    }

    Flywheel(DcMotorEx motorOne, DcMotorEx motorTwo, Telemetry telemetry) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.telemetry = telemetry;
        if (!Double.isFinite(FlywheelConfig.TICKS_PER_REVOLUTION)
                || FlywheelConfig.TICKS_PER_REVOLUTION <= 0
                || !Double.isFinite(FlywheelConfig.GEAR_RATIO) || FlywheelConfig.GEAR_RATIO <= 0
                || !Double.isFinite(FlywheelConfig.MAX_RPM) || FlywheelConfig.MAX_RPM <= 0) {
            stopMotors();
            throw new IllegalArgumentException("Configuracao mecanica da flywheel invalida");
        }
        setGains(FlywheelConfig.KS, FlywheelConfig.KV, FlywheelConfig.KP);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        stopMotors();
    }

    /** Potencia no sentido de lancamento; zero acima da velocidade, sem reversao ativa. */
    public void setMotorPower(double power) {
        double bounded = Double.isFinite(power) ? Math.max(0, Math.min(1, power)) : 0;
        motorOne.setPower(bounded);
        motorTwo.setPower(-bounded);
    }

    public void stopMotors() {
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
        double currentRpm = getRPM();
        if (!Double.isFinite(rpm) || rpm <= 0 || !Double.isFinite(currentRpm)) {
            stopMotors();
            return;
        }
        targetRpm = Math.min(rpm, FlywheelConfig.MAX_RPM);
        double power = kS + kV * targetRpm + kP * (targetRpm - currentRpm);
        setMotorPower(power);
    }

    public double getTicksPerSec() { return motorOne.getVelocity(); }
    public double getRPM() { return toRpm(getTicksPerSec()); }
    public double getSecondMotorRPM() { return toRpm(-motorTwo.getVelocity()); }

    private double toRpm(double ticksPerSecond) {
        return ticksPerSecond * 60 / FlywheelConfig.TICKS_PER_REVOLUTION / FlywheelConfig.GEAR_RATIO;
    }

    public void sendTelemetry() {
        if (telemetry == null) return;
        double currentRpm = getRPM();
        telemetry.addData("Flywheel RPM alvo", targetRpm);
        telemetry.addData("Flywheel RPM medido", currentRpm);
        telemetry.addData("Flywheel RPM motor 2", getSecondMotorRPM());
        telemetry.addData("Flywheel erro RPM", targetRpm - currentRpm);
        telemetry.addData("Flywheel potencia", motorOne.getPower());
    }
}

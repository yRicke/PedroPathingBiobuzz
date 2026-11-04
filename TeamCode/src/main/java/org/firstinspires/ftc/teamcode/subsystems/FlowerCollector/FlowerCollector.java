package org.firstinspires.ftc.teamcode.subsystems.FlowerCollector;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.LongSupplier;

/** Dois servos continuos com abertura/fechamento por tempo, sem bloquear o loop. */
public class FlowerCollector {
    // Unico ajuste de movimento: tempo em segundos, igual para abrir e fechar.
    public static final double SERVO_RUN_TIME_SECONDS = 1.0;

    private final CRServo servoOne;
    private final CRServo servoTwo;
    private final Telemetry telemetry;
    private final LongSupplier nanoTime;
    private Boolean closedRequested;
    private boolean moving;
    private long movementStartedNanos;

    public FlowerCollector(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, System::nanoTime);
    }

    // Relogio substituivel para testar o prazo sem esperas reais.
    FlowerCollector(HardwareMap hardwareMap, Telemetry telemetry, LongSupplier nanoTime) {
        if (!Double.isFinite(SERVO_RUN_TIME_SECONDS) || SERVO_RUN_TIME_SECONDS <= 0) {
            throw new IllegalArgumentException("Tempo do FlowerCollector deve ser positivo e finito");
        }
        this.telemetry = telemetry;
        this.nanoTime = nanoTime;
        servoOne = hardwareMap.get(CRServo.class, "flowerCollect1");
        servoTwo = hardwareMap.get(CRServo.class, "flowerCollect2");
        // Ambos no mesmo sentido inicialmente; inverter cada linha aqui se necessario.
        servoOne.setDirection(DcMotorSimple.Direction.FORWARD);
        servoTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        stop();
    }

    /** Solicita abrir. Se chamado uma vez por evento, mantenha update() no loop. */
    public void open() {
        run(false);
    }

    /** Solicita fechar. Uma inversao durante o movimento inicia um novo prazo completo. */
    public void close() {
        run(true);
    }

    /** true fecha, false abre. Chamadas repetidas do mesmo alvo nao reiniciam o tempo. */
    public void run(boolean closed) {
        if (closedRequested == null || closedRequested != closed) {
            closedRequested = closed;
            movementStartedNanos = nanoTime.getAsLong();
            moving = true;
            setPower(closed ? -1.0 : 1.0);
        }
        update();
    }

    /** Deve ser chamado a cada ciclo para zerar a potencia ao terminar o prazo. */
    public void update() {
        if (moving && elapsedSeconds() >= SERVO_RUN_TIME_SECONDS) {
            setPower(0);
            moving = false;
        }
    }

    /** Cancela o movimento e permite solicitar novamente o mesmo sentido. Use no Stop. */
    public void stop() {
        setPower(0);
        moving = false;
        closedRequested = null;
    }

    public boolean isMoving() {
        return moving;
    }

    public double getRemainingSeconds() {
        return moving ? Math.max(0, SERVO_RUN_TIME_SECONDS - elapsedSeconds()) : 0;
    }

    private double elapsedSeconds() {
        return (nanoTime.getAsLong() - movementStartedNanos) / 1_000_000_000.0;
    }

    private void setPower(double power) {
        servoOne.setPower(power);
        servoTwo.setPower(power);
    }

    public void sendTelemetry() {
        telemetry.addData("FlowerCollector comando", closedRequested == null
                ? "Nenhum" : (closedRequested ? "Fechar" : "Abrir"));
        telemetry.addData("FlowerCollector movimento", moving ? "Rodando" : "Parado");
        telemetry.addData("FlowerCollector tempo restante (s)", "%.2f", getRemainingSeconds());
    }
}

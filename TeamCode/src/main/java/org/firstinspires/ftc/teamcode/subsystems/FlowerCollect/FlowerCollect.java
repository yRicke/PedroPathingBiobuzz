package org.firstinspires.ftc.teamcode.subsystems.FlowerCollect;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Dois servos posicionais com sentidos opostos e posicoes independentes. */
public class FlowerCollect {
    public static final String SERVO_ONE_NAME = "flowerCollect1";
    public static final String SERVO_TWO_NAME = "flowerCollect2";
    public static final double SERVO_ONE_OPEN_POSITION = 1; ;
    public static final double SERVO_TWO_OPEN_POSITION = 1;
    // Provisorias: substituir separadamente pelos valores encontrados no tuner.
    public static final double SERVO_ONE_CLOSED_POSITION = 0;
    public static final double SERVO_TWO_CLOSED_POSITION = 0;

    private final Servo servoOne;
    private final Servo servoTwo;
    private final Telemetry telemetry;
    private double servoOnePosition;
    private double servoTwoPosition;

    public FlowerCollect(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true);
    }

    /** Permite iniciar recolhido sem comandar uma abertura durante o INIT. */
    public FlowerCollect(HardwareMap hardwareMap, Telemetry telemetry, boolean initiallyClosed) {
        this.telemetry = telemetry;
        servoOne = hardwareMap.get(Servo.class, SERVO_ONE_NAME);
        servoTwo = hardwareMap.get(Servo.class, SERVO_TWO_NAME);
        servoOne.setDirection(Servo.Direction.REVERSE

        );
        servoTwo.setDirection(Servo.Direction.FORWARD);
        run(!initiallyClosed);
    }

    public void run(boolean open) {
        if (open) setOpen();
        else setClosed();
    }

    public void setOpen() {
        setPositions(SERVO_ONE_OPEN_POSITION, SERVO_TWO_OPEN_POSITION);
    }

    public void setClosed() {
        setPositions(SERVO_ONE_CLOSED_POSITION, SERVO_TWO_CLOSED_POSITION);
    }

    /** Posicoes logicas; a inversao do segundo servo ja e feita pelo SDK. */
    public void setPositions(double positionOne, double positionTwo) {
        // Valida ambos antes de movimentar qualquer servo.
        double boundedOne = boundPosition(positionOne);
        double boundedTwo = boundPosition(positionTwo);
        setServoOnePosition(boundedOne);
        setServoTwoPosition(boundedTwo);
    }

    public void setServoOnePosition(double position) {
        servoOnePosition = boundPosition(position);
        servoOne.setPosition(servoOnePosition);
    }

    public void setServoTwoPosition(double position) {
        servoTwoPosition = boundPosition(position);
        servoTwo.setPosition(servoTwoPosition);
    }

    public double getServoOnePosition() { return servoOnePosition; }
    public double getServoTwoPosition() { return servoTwoPosition; }

    private static double boundPosition(double position) {
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Posicao do Flower Collect deve ser finita");
        }
        return Math.max(0, Math.min(1, position));
    }

    public void sendTelemetry() {
        telemetry.addData("Flower servo 1 (FORWARD)", "%.3f", servoOnePosition);
        telemetry.addData("Flower servo 2 (REVERSE)", "%.3f", servoTwoPosition);
    }
}

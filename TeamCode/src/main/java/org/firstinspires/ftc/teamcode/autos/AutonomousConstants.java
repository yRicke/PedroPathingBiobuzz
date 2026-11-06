package org.firstinspires.ftc.teamcode.autos;

/** Tempos, frenagem e RPM compartilhados pelos autonomos. */
public final class AutonomousConstants {
    // Ajuste pequeno nos deslocamentos; 1.0 restaura a frenagem padrao.
    public static final double TRANSIT_BRAKE_AGGRESSION = 1.05;
    // Alvo fixo da flywheel somente nos autonomos; a mira continua pela Limelight.
    public static final double FLYWHEEL_RPM = 2560;
    public static final long TERTIARY_INITIAL_DELAY_MS = 8000;
    public static final long SPIN_UP_MS = 1500;
    public static final long AIM_TIME_MS = 500;
    public static final long SHOOT_TIME_MS = 2000;
    public static final long FLOWER_COLLECT_MS = 2000;
    // Servos posicionais nao informam a chegada: reserva tempo para o movimento.
    public static final long FLOWER_SERVO_SETTLE_MS = 500;

    private AutonomousConstants() { }
}

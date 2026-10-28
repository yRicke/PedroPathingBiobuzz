package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

/** Parametros do launcher e pares de calibracao de POLLEN por tdist. */
public final class FlywheelConfig {
    public static final String MOTOR_ONE_NAME = "launcher_motor_one";
    public static final String MOTOR_TWO_NAME = "launcher_motor_two";
    public static final double TICKS_PER_REVOLUTION = 28;
    public static final double GEAR_RATIO = 1;
    public static final double KS = 0.078;
    public static final double KV = 0.0001476;
    public static final double KP = 0.00159;
    public static final double MAX_RPM = 6000;
    public static final double BASE_RPM = 2720;
    public static final double BASE_DISTANCE_CM = 0;
    public static final double BASE_RPM_2 = 0;
    public static final double BASE_DISTANCE_2_CM = 0;
    public static final double BASE_RPM_3 = 0;
    public static final double BASE_DISTANCE_3_CM = 0;
    public static final long SETPOINT_UPDATE_MS = 100;
    public static final double SETPOINT_DEADBAND_RPM = 10;

    private FlywheelConfig() { }

    /** Distancia zero desativa o par ate a medicao no TeleOp de calibracao. */
    public static FlywheelRpmTable createRpmTable() {
        return new FlywheelRpmTable(BASE_RPM, MAX_RPM, new double[][] {
                {BASE_DISTANCE_CM, BASE_RPM},
                {BASE_DISTANCE_2_CM, BASE_RPM_2},
                {BASE_DISTANCE_3_CM, BASE_RPM_3}
        });
    }
}

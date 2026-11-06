package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/** Configuracoes dos autonomos principal e secundario: poses em polegadas e graus. */
public final class AutonomousConstants {
    private static final PoseFactory POSES = PoseFactory.degrees();

    // Ajuste pequeno nos deslocamentos; 1.0 restaura a frenagem padrao.
    public static final double TRANSIT_BRAKE_AGGRESSION = 1.05;
    public static final long SPIN_UP_MS = 1500;
    public static final long AIM_TIME_MS = 500;
    public static final long SHOOT_TIME_MS = 2000;
    public static final long FLOWER_COLLECT_MS = 2000;
    // Servos posicionais nao informam a chegada: reserva tempo para o movimento.
    public static final long FLOWER_SERVO_SETTLE_MS = 500;

    // Indices na sequencia de 11 trechos dos dois autonomos.
    public static final int SECOND_SHOT_PATH = 3;
    public static final int PREP_FLOWER_PATH = 5;
    public static final int COLLECT_FLOWER_PATH = 6;
    public static final int POST_FLOWER_PATH = 7;
    public static final int THIRD_SHOT_PATH = 9;
    private static final String[] PATH_NAMES = {
            "chegadinha_frente_1", "prep_coleta_jardim", "coleta_jardim", "lancamento_2",
            "chegadinha_frente_2", "prep_coleta_flower", "coleta_flower", "pos_flower",
            "pre_lancamento_3", "lancamento_3", "estacionamento"
    };

    private AutonomousConstants() { }

    static String pathName(int index) {
        return PATH_NAMES[index];
    }

    /** Poses do percurso em paths/AutoPrincipal.pp. */
    public static final class Principal {
        public static final Pose START = POSES.of(57.8, 9, 90);
        public static final Pose CHEGADINHA_FRENTE_1 = POSES.of(57.8, 20, 90);
        public static final Pose PREP_COLETA_JARDIM = POSES.of(31, 13, 180);
        public static final Pose COLETA_JARDIM = POSES.of(14, 13, 180);
        public static final Pose LANCAMENTO_2 = POSES.of(60, 147, 270);
        public static final Pose LANCAMENTO_CONTROL_1 = POSES.of(38.1594, 48.3839, 0);
        public static final Pose LANCAMENTO_CONTROL_2 = POSES.of(6.5729, 108.8181, 0);
        public static final Pose LANCAMENTO_CONTROL_3 = POSES.of(57.3303, 105.8968, 0);
        public static final Pose CHEGADINHA_FRENTE_2 = POSES.of(60, 122, 270);
        public static final Pose PREP_COLETA_FLOWER = POSES.of(50.6, 122, 90);
        public static final Pose COLETA_FLOWER = POSES.of(46.6, 131, 90);
        public static final Pose POS_FLOWER = POSES.of(46.6, 120, 90);
        public static final Pose PREP_LANCAMENTO_3 = POSES.of(60, 122, 270);
        public static final Pose LANCAMENTO_3 = POSES.of(60, 147, 270);
        public static final Pose ESTACIONAMENTO = POSES.of(10, 120, 0);
        public static final Pose ESTACIONAMENTO_CONTROL_1 = POSES.of(65.729, 114.1129, 0);

        private Principal() { }
    }

    /** Poses do percurso em paths/AutoSecundario.pp.json, com dois controles no lancamento 2. */
    public static final class Secundario {
        public static final Pose START = POSES.of(57.8, 9, 90);
        public static final Pose CHEGADINHA_FRENTE_1 = POSES.of(57.8, 20, 90);
        public static final Pose PREP_COLETA_JARDIM = POSES.of(31, 13, 180);
        public static final Pose COLETA_JARDIM = POSES.of(14, 13, 180);
        public static final Pose LANCAMENTO_2 = POSES.of(60, 147, 270);
        public static final Pose LANCAMENTO_CONTROL_1 = POSES.of(78.32714193548385, 1.825835483870963, 0);
        public static final Pose LANCAMENTO_CONTROL_2 = POSES.of(56.41741612903226, 50.757454838709684, 0);
        public static final Pose CHEGADINHA_FRENTE_2 = POSES.of(60, 122, 270);
        public static final Pose PREP_COLETA_FLOWER = POSES.of(50.6, 122, 90);
        public static final Pose COLETA_FLOWER = POSES.of(46.6, 131, 90);
        public static final Pose POS_FLOWER = POSES.of(46.6, 120, 90);
        public static final Pose PREP_LANCAMENTO_3 = POSES.of(60, 122, 270);
        public static final Pose LANCAMENTO_3 = POSES.of(60, 147, 270);
        public static final Pose ESTACIONAMENTO = POSES.of(10, 120, 0);
        public static final Pose ESTACIONAMENTO_CONTROL_1 = POSES.of(65.729, 114.1129, 0);

        private Secundario() { }
    }
}

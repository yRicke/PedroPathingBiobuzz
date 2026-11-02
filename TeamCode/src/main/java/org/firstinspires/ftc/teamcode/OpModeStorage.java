package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/** Transfere a ultima pose medida do autonomo para um unico TeleOp no mesmo processo. */
public final class OpModeStorage {
    private static Pose autonomousEndPose;

    private OpModeStorage() {}

    /** Todo novo autonomo invalida a pose anterior, inclusive se for cancelado no init. */
    public static synchronized void clearAutonomousPose() {
        autonomousEndPose = null;
    }

    public static synchronized void saveAutonomousPose(Pose pose) {
        autonomousEndPose = pose;
    }

    /** Retorna null quando nao ha autonomo disponivel; a leitura consome a pose. */
    public static synchronized Pose consumeAutonomousPose() {
        Pose pose = autonomousEndPose;
        autonomousEndPose = null;
        return pose;
    }
}

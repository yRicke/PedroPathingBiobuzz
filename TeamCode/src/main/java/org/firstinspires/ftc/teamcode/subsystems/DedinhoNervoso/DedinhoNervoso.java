package org.firstinspires.ftc.teamcode.subsystems.DedinhoNervoso;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DedinhoNervoso {

    private Servo DedinhoNervoso = null;

    private Telemetry telemetry;

    public final double DedinhoNervoso_OPEN_POSITION = 0.1; // Posição aberta do gatilho
    public final double DedinhoNervoso_CLOSED_POSITION = 0.4; // Posição fechada do gatilho

    // Construtor recebe o hardwareMap e telemetry do OpMode
    public DedinhoNervoso(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        DedinhoNervoso = hardwareMap.get(Servo.class, "DedinhoNervoso");
    }

    // Executa o intake com base no gatilho
    public void run(boolean DedinhoNervoso_on) {
        if (DedinhoNervoso_on) {
            setDedinhoNervosoOpen();
        }
        else{
            setDedinhoNervosoClosed();
        }
    }

    public void setDedinhoNervosoOpen(){
        DedinhoNervoso.setPosition(DedinhoNervoso_OPEN_POSITION); // Abre o gatilho
    }
    public void setDedinhoNervosoClosed(){
        DedinhoNervoso.setPosition(DedinhoNervoso_CLOSED_POSITION); // Fecha o gatilho
    }

    // Telemetria
    public void sendTelemetry() {
        double servoPosition = DedinhoNervoso.getPosition();
        telemetry.addData("DedinhoNervoso State", servoPosition == DedinhoNervoso_CLOSED_POSITION ? "Closed" : "Open");
        telemetry.addData("DedinhoNervoso Position", servoPosition);
    }
}

# Flywheel

`Flywheel` é o subsistema de hardware: construtor com `HardwareMap` e
`Telemetry`, `run(rpm)` a cada ciclo, `sendTelemetry()` e `stopMotors()`.

## Configuração

Todos os parâmetros compartilhados ficam em `FlywheelConfig.java`.

| Parâmetro | Uso |
| --- | --- |
| `MOTOR_ONE_NAME`, `MOTOR_TWO_NAME` | Nomes no Robot Controller |
| `TICKS_PER_REVOLUTION`, `GEAR_RATIO` | Conversão do encoder para RPM |
| `KS`, `KV`, `KP` | Feedforward e proporcional |
| `MAX_RPM` | Limite configurado do launcher |
| `BASE_RPM`, `BASE_DISTANCE_CM` | Primeiro par por tdist; RPM base também é usada sem leitura |
| `BASE_RPM_2/3`, `BASE_DISTANCE_2/3_CM` | Pares adicionais |
| `SETPOINT_UPDATE_MS`, `SETPOINT_DEADBAND_RPM` | Estabilização da RPM automática |

O controle usa `KS + KV × alvo + KP × (alvo − medido)`, limitado a [0, 1].
Ao exceder a velocidade, reduz a potência até zero. O motor 2 recebe o sinal
físico correspondente à sua montagem. O encoder do motor 1 fecha a malha;
a telemetria também mostra a RPM normalizada do segundo motor.

## Ajustes do controlador

Os OpModes de `Tunning/` usam D-pad esquerda/direita para escolher o passo e
cima/baixo para mudar o ganho. A/B escolhem 2800/2200 RPM nos tuners de kV/kP.
Os ganhos iniciam com os valores de `FlywheelConfig`.

1. `kS Tuner`: aplica potência diretamente para medir o atrito estático.
2. `kV Tuner`: usa `Flywheel.run()` com KP zero para ajustar o feedforward.
3. `kP Tuner`: usa `Flywheel.run()` com os três ganhos para ajustar a resposta.
4. Copie os ganhos aprovados para `FlywheelConfig` e gere o programa novamente.

Os tuners desligam a flywheel no método `stop()` do OpMode. O `Flywheel Test`
em `Tests/` liga a flywheel enquanto o gatilho direito estiver pressionado.

## RPM por distância

`FlywheelRpmTable` é imutável: valida e ordena os pares na construção.
`FlywheelRpmController` mantém o estado e a frequência de atualização do
setpoint; o TeleOp só fornece a distância e encaminha o resultado à flywheel.
A RPM manual do TeleOp de calibração passa diretamente ao controle dos motores.

Veja [o procedimento por tdist](../../../../../../../../../../docs/flywheel-tdist-calibration.md).

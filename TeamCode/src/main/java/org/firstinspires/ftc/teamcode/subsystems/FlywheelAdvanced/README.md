# Flywheel

O subsistema recebe `HardwareMap` e `Telemetry`, controla os motores e publica
seu estado. As configurações ficam no início de `Flywheel.java`.

| Parâmetro | Uso |
| --- | --- |
| `MOTOR_ONE_NAME`, `MOTOR_TWO_NAME` | Nomes no Robot Controller |
| `TICKS_PER_REVOLUTION`, `GEAR_RATIO` | Conversão do encoder para RPM |
| `KS`, `KV`, `KP` | Feedforward e proporcional |
| `MAX_RPM` | Limite configurado do launcher |
| `BASE_RPM`, `BASE_DISTANCE_CM` | Primeiro par por distância 3D ao centro da CELL; RPM base também é usada sem leitura |
| `BASE_RPM_2/3`, `BASE_DISTANCE_2/3_CM` | Pares adicionais |
| `SETPOINT_UPDATE_MS`, `SETPOINT_DEADBAND_RPM` | Estabilização da RPM automática |

## Uso

```java
Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
flywheel.runByDistance(limelight.getDistanceMeters()); // automático, a cada ciclo
flywheel.sendTelemetry();
```

Para ajuste manual/teste, use `flywheel.run(rpm)` no lugar de `runByDistance()`.
Esse modo permite aplicar passos de 0,1 RPM sem banda morta. Trocar entre os
modos limpa o estado necessário; `stopMotors()` desliga e reinicia o setpoint.
`getTargetRPM()` informa a RPM aplicada, e `getRPM()` a velocidade medida.

O controle usa `KS + KV × alvo + KP × (alvo − medido)`, limitado a [0, 1].
Ao exceder a velocidade, reduz potência até zero. O motor 2 recebe o sinal
físico correspondente à montagem. O encoder 1 fecha a malha; a telemetria
mostra também a RPM normalizada do segundo motor.

`FlywheelRpmTable` é a classe separada responsável apenas por copiar, validar,
ordenar e interpolar os pares. A consulta da tabela, a banda morta e o intervalo
de atualização são métodos internos da própria `Flywheel`.

## Ajustes do controlador

Os OpModes de `Tunning/` usam D-pad esquerda/direita para o passo e cima/baixo
para o ganho. A/B escolhem 2800/2200 RPM nos tuners de kV/kP. Os ganhos iniciam
com as constantes da própria `Flywheel`.

1. `kS Tuner`: aplica potência diretamente para medir o atrito estático.
2. `kV Tuner`: usa `Flywheel.run()` com KP zero para ajustar o feedforward.
3. `kP Tuner`: usa `Flywheel.run()` com os três ganhos para ajustar a resposta.
4. Copie os ganhos aprovados para as constantes no início de `Flywheel.java`.

Os tuners desligam os motores no `stop()`. `Flywheel Test`, em `Tests/`, liga
os motores enquanto o gatilho direito estiver pressionado.

Veja [o procedimento por distância 3D](../../../../../../../../../../docs/flywheel-tdist-calibration.md).

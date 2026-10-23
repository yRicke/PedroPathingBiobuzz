# Organização dos subsistemas

Cada pasta reúne as classes do subsistema, seus OpModes de teste em `Tests/`
e seus OpModes de ajuste em `Tunning/`, quando existentes.
Os grupos no Driver Station seguem o nome do subsistema.

| Pasta | Classes | Grupo no Driver Station |
| --- | --- | --- |
| `AprilTagWebcam/` | `AprilTagWebcam` | `AprilTagWebcam` |
| `Drive/` | `Drive`, `DriveField`, `PedroDrive` | `Drive` |
| `FlywheelAdvanced/` | `Flywheel` | `Flywheel` |
| `Intake/` | `Intake` | `Intake` |
| `Limelight/` | `Limelight`, `AprilTagObservation` | Integrado ao TeleOp |
| `Launcher/` | `LauncherPower`, `LauncherVelocity`, `LauncherPIDado` | `Launcher` |
| `Trigger/` | `Trigger` | `Trigger` |
| `Voltage/` | `Voltage` | `Voltage` |

O `TeleOpMainTransferTest` usa `PedroDrive` com o `Follower` criado por
`pedro.Constants`, aproveitando a localização do Pinpoint e o mecanum já configurados.
O controle segue a [documentação oficial de TeleOp do Pedro Pathing](https://pedropathing.com/docs/pathing/guide/teleop-usage):
`ManualDrive.fieldCentric()` transforma a translação pelo heading e
`ManualDrive.driveOrHold()` mantém a posição e a orientação quando os sticks são
soltos e o robô para. Mover os sticks retoma o controle manual.

A orientação ao apertar **Play** define a frente do campo. O gatilho esquerdo do
gamepad 1 mantém o modo precisão com 30% da potência, e seu stick direito horizontal
controla o giro. A telemetria mostra o modo (`MANUAL`/`HOLD`),
a posição em polegadas e o heading em graus.

## Mira com Limelight

No `TeleOpMainTransferTest`, configure `LIMELIGHT_UPSIDE_DOWN` como `false`
para montagem normal ou `true` para a câmera fisicamente de cabeça para baixo.
Configure o dispositivo como `limelight` no Robot Controller e prepare uma
pipeline **AprilTag** com os IDs 39 e 43 habilitados. O índice padrão é **0**,
alterável em `LIMELIGHT_PIPELINE` no TeleOp.

A seleção é feita por `limelight.run(gamepad1.dpad_left, gamepad1.dpad_right)`:

- D-pad esquerdo seleciona **39**; direito seleciona **43**. A seleção inicial é 43.
- A seleção permanece ao soltar o D-pad; pressionar os dois lados mantém a anterior.
- Segurar o **right trigger do gamepad 1** ativa **mira e disparo juntos**.
  O Pedro captura X/Y, ignora os sticks da base durante a mira e altera apenas
  o heading para centralizar a tag selecionada, com tolerância inicial de 1 grau.
- Sem enxergar essa tag, busca com giro lento, inicialmente horário. Depois de
  detectar a tag, a busca segue a direção do último erro observado.
- Desconexão, pipeline incorreta e imagens antigas/congeladas interrompem a busca
  e mantêm a pose. O disparo continua obedecendo ao gatilho, inclusive sem visão.
- Soltar o gatilho cancela o alvo de rotação e devolve o drive field-centric.

O subsistema usa o erro horizontal da tag selecionada sem deslocamento do
crosshair. A montagem invertida corrige esse sinal no código. A opção de
[orientação do stream](https://docs.limelightvision.io/docs/docs-limelight/getting-started/pipelines)
da interface da câmera altera a exibição, sem corrigir os dados de detecção.
Frames repetidos não acumulam correções; dados com mais de 150 ms ou timestamp
congelado são rejeitados. O limite de correção inicial é de 8 graus por imagem.

Para o teste físico, selecione cada tag pelo D-pad e observe `AprilTag selecionada`
e `AprilTag erro horizontal (deg)` na telemetria. Com a tag deslocada para a direita,
o robô deve girar para a direita e reduzir o erro até perto de zero. Confira esse
sentido também na montagem invertida. Segure o gatilho para verificar a manutenção
de X/Y e o acompanhamento do alvo; solte para conferir a retomada pelos sticks.
A telemetria `Mira` distingue busca, centralização, alinhamento e ausência de visão.

Os valores de tolerância, passo de busca e limite de correção ficam no início de
`Drive/PedroDrive.java` para ajuste no robô. O controle de pose usa o Foresight
e a calibração do Pinpoint já existentes em `pedro.Constants`.

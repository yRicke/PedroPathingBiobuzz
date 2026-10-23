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
gamepad 1 mantém o modo precisão com 30% da potência, e o stick direito horizontal
do gamepad 2 mantém a correção de giro. A telemetria mostra o modo (`MANUAL`/`HOLD`),
a posição em polegadas e o heading em graus.

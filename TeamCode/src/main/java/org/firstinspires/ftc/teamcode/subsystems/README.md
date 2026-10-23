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
| `Limelight/` | `Limelight`, `AprilTagObservation`, `HiveCenterEstimator` | Integrado ao TeleOp |
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
pipeline **AprilTag 36h11** com os IDs **38 a 45** habilitados. O índice fica
em `LIMELIGHT_PIPELINE` no TeleOp. Ative **Full3D** nas opções avançadas e configure
o tamanho da tag como **3,25 polegadas (82,55 mm)**. A pose 3D individual é necessária
para estimar o centro e a distância, conforme a
[API oficial da Limelight para FTC](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).

A seleção é feita por `limelight.run(gamepad1.dpad_left, gamepad1.dpad_right)`:

- D-pad esquerdo seleciona o hive **38, 39, 40, 41**; direito seleciona
  **42, 43, 44, 45**. O hive direito é a seleção inicial.
- A seleção permanece ao soltar o D-pad; pressionar os dois lados mantém a anterior.
- Segurar o **right bumper do gamepad 1** ativa a **busca e a mira**.
  O Pedro captura X/Y, ignora os sticks da base durante a mira e altera apenas
  o heading para mirar no **centro estimado do hive**, com tolerância inicial de 1 grau.
- Basta enxergar **uma** das quatro tags do lado selecionado. Sem enxergar nenhuma,
  busca com giro lento, inicialmente horário. Depois de detectar o hive, a busca
  segue a direção do último erro observado.
- O **right trigger do gamepad 1** aciona o disparo, independentemente da mira.
- Desconexão, pipeline incorreta e imagens antigas/congeladas interrompem a busca
  e mantêm a pose. O disparo continua obedecendo ao gatilho, inclusive sem visão.
- Soltar o right bumper cancela o alvo de rotação e devolve o drive field-centric.

O `HiveCenterEstimator` aplica à pose 3D de cada tag o deslocamento até o centro
do seu hive, considerando a rotação da tag. Pela figura 9-15 do
[manual oficial BIOBUZZ, seção 9](https://ftc-resources.firstinspires.org/ftc/game/manual-09),
as posições horizontais das tags em relação ao centro são **−6,5; −2,75; +2,75;
+6,5 polegadas**, na ordem crescente dos IDs de cada lado. O espaçamento não é uniforme.
Quando várias tags estão visíveis, combina as posições do centro estimadas por
cada uma, ponderando pela área da tag na imagem. Não calcula a média dos centros
das tags visíveis, pois isso deslocaria a mira quando uma tag desaparecesse.

O ângulo do centro é calculado no
[espaço da câmera](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems),
com origem na lente, X para a direita, Y para baixo e Z para a frente.
A montagem invertida corrige o sinal horizontal no código. A opção de
[orientação do stream](https://docs.limelightvision.io/docs/docs-limelight/getting-started/pipelines)
da interface da câmera altera a exibição, sem corrigir os dados de detecção.
Frames repetidos não acumulam correções; dados com mais de 150 ms ou timestamp
congelado são rejeitados. O limite de correção inicial é de 8 graus por imagem.

Depois de chamar `run()`, `limelight.getDistanceMeters()` retorna um `OptionalDouble`
com a **distância em linha reta da lente ao centro estimado**, em metros, incluindo
a diferença de altura. O valor fica vazio sem leitura válida; não retorna zero nem
reutiliza a distância anterior. A telemetria mostra `Distancia ate centro hive (cm)`,
`Hive selecionado`, `Tags usadas no centro` e `Centro hive erro horizontal (deg)`.
Se uma tag do hive aparece sem pose 3D válida, mostra a orientação para conferir
Full3D e mantém a pose do robô.

Para o teste físico, selecione cada hive pelo D-pad e observe os valores na telemetria.
Cubra três tags e confira se a mira continua no mesmo centro usando qualquer uma
das quatro. Compare a distância com uma medida da lente até esse centro.
Com o centro deslocado para a direita,
o robô deve girar para a direita e reduzir o erro até perto de zero. Confira esse
sentido também na montagem invertida. Segure o right bumper para verificar a manutenção
de X/Y e o acompanhamento do alvo; solte para conferir a retomada pelos sticks.
A telemetria `Mira` distingue busca, centralização, alinhamento e ausência de visão.

Os valores de tolerância, passo de busca e limite de correção ficam no início de
`Drive/PedroDrive.java` para ajuste no robô. O controle de pose usa o Foresight
e a calibração do Pinpoint já existentes em `pedro.Constants`.

## RPM pela distância ao hive

No `TeleOpMainTransferTest`, as variáveis públicas `RPM_BASE = 2700` e
`DISTANCIA_BASE_CM = 308` definem a referência de lançamento. Ajuste esses valores
no início da classe para alterar a calibração. A cada ciclo, o TeleOp passa a
distância atual da Limelight ao `FlywheelRpmCalculator` e atualiza o alvo do flywheel:

`RPM alvo = RPM_BASE × (distância em cm / DISTANCIA_BASE_CM)`

O getter da Limelight retorna metros; o calculador converte para centímetros.
Com os valores iniciais, 154 cm produzem 1350 RPM, 308 cm produzem 2700 RPM e
462 cm produzem 4050 RPM. Esse é um modelo proporcional inicial para testar;
é necessário conferir os lançamentos em outras distâncias no robô.

Sem distância válida, o alvo volta imediatamente a `RPM_BASE`. Uma referência de
distância inválida também usa o RPM base; um RPM base zero, negativo ou não finito
desliga os motores. A telemetria existente `Flywheel RPM alvo` mostra o valor
calculado, junto de `Flywheel RPM medido` e da distância ao centro do hive.
O ajuste independe do right bumper: ele controla a mira, enquanto o right trigger
continua acionando o disparo.

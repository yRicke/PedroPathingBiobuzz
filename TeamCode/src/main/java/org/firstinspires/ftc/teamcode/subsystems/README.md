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
- Segurar o **right bumper do gamepad 1** ativa a mira no **centro estimado do hive**,
  com tolerância inicial de 1 grau. O stick esquerdo continua controlando X/Y
  em field-centric; somente o giro é substituído pela correção visual.
- Basta enxergar **uma** das quatro tags do lado selecionado. Sem enxergar nenhuma,
  o giro volta imediatamente ao stick direito, mesmo com o bumper pressionado.
  A mira retoma quando o hive selecionado reaparece; não há giro de busca automático.
- O **right trigger do gamepad 1** aciona o disparo, independentemente da mira.
- Desconexão, pipeline incorreta e imagens antigas/congeladas também devolvem
  o giro ao piloto. O disparo continua obedecendo ao gatilho, inclusive sem visão.
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
a diferença de altura, filtrada pela média aparada descrita abaixo. A leitura
instantânea continua em `limelight.getObservation().getDistanceMeters()`.
O getter filtrado fica vazio sem leitura válida. A telemetria mostra a distância
filtrada em `Distancia ate centro hive (cm)`, a leitura bruta em
`Distancia instantanea centro hive (cm)` e `Amostras distancia flywheel`, além de
`Hive selecionado`, `Tags usadas no centro` e `Centro hive erro horizontal (deg)`.
Se uma tag do hive aparece sem pose 3D válida, mostra a orientação para conferir
Full3D e devolve o giro ao piloto.

Para o teste físico, selecione cada hive pelo D-pad e observe os valores na telemetria.
Cubra três tags e confira se a mira continua no mesmo centro usando qualquer uma
das quatro. Compare a distância com uma medida da lente até esse centro.
Com o centro deslocado para a direita,
o robô deve girar para a direita e reduzir o erro até perto de zero. Confira esse
sentido também na montagem invertida. Segure o right bumper e mova o stick esquerdo:
X/Y devem responder enquanto o heading acompanha o centro. Cubra todas as tags e
confira o giro pelo stick direito sem soltar o bumper. Descubra as tags para retomar
a mira e solte o bumper para conferir o giro manual. Com sticks neutros e sem mira,
o hold automático normal continua disponível.
A telemetria `Mira` distingue centralização, alinhamento e giro manual sem alvo/visão.

Os valores de tolerância e limite de correção ficam no início de
`Drive/PedroDrive.java` para ajuste no robô. A mira usa `ManualDrive.headingLock()`
com o controlador de heading calibrado em `pedro.Constants`, limita a correção
a 8 graus por imagem nova e mantém o alvo em imagens repetidas. O modo precisão
também reduz a translação e o giro durante a mira para 30%.

## Calibração e RPM pelo tdist

Execute `Flywheel Calibration - tdist` para ajustar manualmente a RPM e medir
o `tdist`. O procedimento completo e os controles estão em
[docs/flywheel-tdist-calibration.md](../../../../../../../../../docs/flywheel-tdist-calibration.md).

O `TeleOpMainTransferTest` agora usa `limelight.getTdistMeters()`, separado de
`getDistanceMeters()` e da pose usada na mira. O SDK 12.0.0 conserva os campos
novos no JSON retornado por `LLResult.toString()`; `TdistReader` lê `tdist` desse
JSON, sem requisições HTTP adicionais. Não há fallback para a distância antiga.

Preencha `DISTANCIA_BASE_CM`/`RPM_BASE`, `DISTANCIA_BASE_2_CM`/`RPM_BASE_2` e
`DISTANCIA_BASE_3_CM`/`RPM_BASE_3` no início do TeleOp com pares experimentais.
Distância zero desativa o par. Inicialmente as distâncias são zero, pois os
antigos 308 cm não foram medidos com `tdist`. `RPM_BASE` começa em 2720.

Com zero pares, usa RPM_BASE; com um, mantém a RPM do único par. Com dois ou
mais, interpola linearmente entre os pares. Fora da faixa, limita a RPM ao
extremo mais próximo e sinaliza na telemetria. Isso não valida o disparo fora
da faixa. Distâncias duplicadas ou pares ativos inválidos são rejeitados antes
de iniciar o hardware.

Os getters retornam metros; o calculador converte para centímetros.
O filtro de `tdist` tem histórico separado da distância antiga.
O `Limelight/CenterDistanceFilter` mantém uma janela móvel de **1 segundo**, com
**até 40 amostras** válidas e novas, separadas por pelo menos **25 ms**. Frames
repetidos não aumentam a contagem. Ordena as distâncias, descarta os **25% menores
e 25% maiores** e calcula a média das restantes: com 40, usa as **20 centrais**.
Na partida usa as amostras disponíveis, arredondando o descarte para baixo; não
bloqueia o loop para preencher a janela. A frequência real depende dos frames
novos da câmera e da frequência do TeleOp.

O histórico é limpo ao trocar de hive, perder o alvo, receber dados inválidos ou
parar a Limelight. A mira usa o ângulo instantâneo, sem o atraso do filtro de distância.
Para ajustar a janela e a quantidade de amostras, altere `WINDOW_NANOS` e
`MAX_SAMPLES` em `CenterDistanceFilter`.

Sem `tdist` válido, o pedido volta a `RPM_BASE`; um RPM base zero, negativo ou não
finito desliga os motores. Alterações do setpoint são aplicadas no máximo a cada
100 ms e somente com diferença de pelo menos 10 RPM. O controlador da flywheel
continua rodando em todos os ciclos. `Flywheel RPM alvo` mostra o alvo aplicado;
`RPM calculada antes da banda morta` mostra o pedido antes dessa estabilização.
O ajuste independe do right bumper: ele controla a mira, enquanto o right trigger
continua acionando o disparo.

# Organização dos subsistemas

Cada pasta reúne o subsistema, seus OpModes de teste em `Tests/` e seus ajustes
em `Tunning/`, quando existentes. Os subsistemas recebem `HardwareMap` e
`Telemetry`, executam `run(...)` a cada ciclo e publicam `sendTelemetry()`.
O TeleOp coordena entradas e chamadas dos subsistemas.

| Pasta | Responsabilidade |
| --- | --- |
| `Drive/` | Drive manual, field-centric, hold e mira pela Limelight |
| `Limelight/` | Câmera, validação de frames e abertura da CELL no referencial do lançador |
| `FlywheelAdvanced/` | Motores, consulta da tabela e estabilização da RPM |
| `Intake/` | Coleta e ejeção |
| `Transfer/` | Alimentação dos elementos |
| `Trigger/` | Abertura do disparo |
| `DedinhoNervoso/` | Retenção/liberação pelo servo |
| `FlowerCollect/` | Dois servos espelhados, posições independentes e TeleOp de ajuste |

## Flower Collect

O `FlowerCollect` usa os servos posicionais `flowerCollect1` e `flowerCollect2`,
em portas separadas. O primeiro usa `FORWARD` e o segundo `REVERSE`. As posições
lógicas são independentes e limitadas entre 0 e 1; a inversão do segundo já é
aplicada pelo SDK. As quatro constantes no início de `FlowerCollect.java`
definem aberto/fechado para cada servo: aberto começa em 0 e fechado em 1,
provisoriamente. `run(true)` fecha, `run(false)` abre. `setPositions(...)` ou
os métodos `setServoOnePosition(...)` e `setServoTwoPosition(...)` permitem
ajustes independentes. Ainda não está integrado ao `TeleOpMain`.

Selecione **Flower Collect Tuner**, grupo **FlowerCollect**, na Driver Station.
No controle 1, **A** seleciona o servo 1 e **B** seleciona o servo 2. D-pad
cima/baixo aumenta/diminui somente a posição selecionada; esquerda/direita
diminui/aumenta o passo entre **0.001, 0.01 e 0.1**. Começa com servo 1 e passo
0.001. Cada toque faz um ajuste; segurar não repete. A seleção mantém a posição
do outro servo. A telemetria mostra o servo selecionado, o passo e as duas
posições comandadas (não são medições físicas). Anote os valores e copie cada
um para sua constante correspondente; o tuner não salva a calibração.

O goBILDA **2000-0025-0502** precisa estar no modo de posição, que é o padrão
de fábrica. No Servo Programmer, selecione a chave **S** e pressione **P**;
os LEDs piscam ao confirmar. O modo de rotação contínua controla velocidade e
não serve para este tuner. Consulte a [ficha do servo](https://www.gobilda.com/2000-series-5-turn-dual-mode-servo-25-2-torque/)
e as [instruções do programador](https://www.gobilda.com/servo-programmer-for-2000-series-dual-mode-servo/).

Esse modelo pode percorrer até cinco voltas (1800°), dependendo da faixa PWM
do controlador. O código mantém a faixa PWM padrão do SDK. **Antes do primeiro
INIT, desacople os braços do mecanismo:** a inicialização comanda ambos para
0 lógico, com sentidos opostos. Só depois alinhe os braços na abertura
desejada, sem forçar batentes. Ajuste um lado por vez com passo 0.001. A posição
fechada 1 é provisória e pode estar várias voltas distante; encontre o valor
de cada lado antes de usar `setClosed()` no mecanismo montado.

## TeleOp principal

`TeleOpMain` usa o `Follower` criado por `pedro.Constants`, com
Pinpoint e mecanum. Recebe a pose do autônomo no INIT ou usa `(8.4, 35.2, 0°)`;
o Play preserva o heading global. Y redefine somente a frente dos controles
e X relocaliza na pose conhecida e restaura a referência global.
O stick esquerdo controla a translação field-centric, o direito controla o giro
e o gatilho esquerdo aplica modo precisão. Ao parar e soltar os sticks, o
Pedro mantém a pose. Right bumper ativa a mira pela Limelight com translação
livre após três frames novos da CELL selecionada. Sem alvo confirmado, o giro
volta imediatamente ao piloto; com sticks neutros conserva a pose atual.
A mira não faz busca por coordenadas da HIVE. Right trigger continua acionando
transfer/trigger.

Depois de atualizar a Limelight, o TeleOp chama:

```java
flywheel.runByDistance(limelight.getDistanceMeters());
```

A mira recebe somente `limelight.getObservation()` por `drive.aimVision(...)`.
A seleção de aliança/CELL determina quais tags serão utilizadas.
No modo manual/teste, a flywheel
recebe uma RPM diretamente por `flywheel.run(rpm)`.

## Quatro classes principais

| Classe | Papel |
| --- | --- |
| `Limelight` | Câmera, configurações, abertura 3D, geometria do lançador e filtragem |
| `AprilTagObservation` | Dados imutáveis de heading publicados para o drive |
| `Flywheel` | Configurações, motores, RPM manual/automática, banda morta e telemetria |
| `FlywheelRpmTable` | Tabela imutável de pares medidos, validação e interpolação |

O TeleOp de calibração mantém seus próprios controles e passos de RPM.
Os ajustes de kS/kV/kP e os OpModes de teste ficam nas suas pastas.

## Limelight

As constantes ficam no início de `Limelight.java`: dispositivo, pipeline,
montagem, polling, idade máxima, janela e IDs. No `TeleOpMain`, D-pad do gamepad 2 cima
seleciona azul e baixo, vermelho. Esquerda seleciona a CELL da plateia (azul
38–41 / vermelho 34–37); direita seleciona a oposta (azul 42–45 / vermelho
30–33). Inicial: azul 42–45. A telemetria mostra aliança, CELL e IDs.
O método privado `estimateOpening()` projeta a abertura pelas poses das tags,
usando a geometria oficial do SDK FTC 12.0.0 e a montagem da câmera.
Combina as estimativas da abertura ponderadas pela área das tags.
A observação contém estado da mira, ângulo, quantidade de tags e timestamp.

`estimateOpening()` aplica a rotação e a posição da câmera em relação à saída
das bolas e calcula a distância 3D até a abertura. Usa as poses individuais do SDK FTC
12.0.0, disponíveis no OS 2026.0 com Full3D, sem depender de `tdist`, POI ou
tag principal. `getRawDistanceMeters()` expõe a leitura bruta e
`getDistanceMeters()` a filtrada, em metros. A telemetria mostra
`Distancia abertura filtrada (cm)`. Tags fora da CELL selecionada são ignoradas.
Pares antigos medidos por `tdist`/POI ou da câmera ao adesivo precisam ser recalibrados.

A janela circular interna mantém até 40 frames distintos em um segundo,
separados por pelo menos 25 ms. Descarta 25% dos menores e dos maiores e
calcula a média restante (20 centrais com 40 amostras). Só recalcula quando a
janela muda. Perda de leitura, troca de HIVE ou reinício do timestamp limpa
o histórico, inclusive ao trocar de aliança. Imagens antigas, congeladas, pipeline incorreta e desconexão são
rejeitadas. A decodificação e a estimativa da abertura ocorrem uma vez por frame/CELL.

Veja o [guia do painel da Limelight e das AprilTags BIOBUZZ](Limelight/README.md),
incluindo os ID Filters e os cuidados com as CELLS móveis.

## Flywheel

As constantes ficam no início de `Flywheel.java`: nomes dos motores, ticks por
revolução, transmissão, ganhos, limite de RPM, pares medidos e estabilização.
A tabela copia, valida e ordena os pares na inicialização. `runByDistance()`
consulta a tabela e aplica banda morta de 10 RPM e intervalo mínimo de 100 ms.
Ao perder a distância, solicita imediatamente a RPM base. Sem pares/um par
usa RPM fixa; com dois ou mais interpola e limita aos extremos, sinalizando
posições fora da faixa.

`run(rpm)` aplica o ajuste manual diretamente, inclusive passos de 0,1 RPM.
Nos dois modos a malha dos motores é executada em todos os ciclos. O controle
usa feedforward mais proporcional e potência entre 0 e 1 no sentido de
lançamento. Overspeed reduz potência até zero. O encoder 1 fecha a malha;
o encoder 2 aparece na telemetria para diagnóstico. RPM/leitura principal
não finita desliga ambos os motores. `run(0)` e `stopMotors()` desligam.

Os tuners de kV/kP usam o mesmo controlador e os ganhos compartilhados de
`Flywheel`. As medições aprovadas devem ser copiadas para suas constantes.

Veja o [procedimento de calibração](../../../../../../../../../docs/flywheel-tdist-calibration.md).

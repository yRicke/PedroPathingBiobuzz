# Organização dos subsistemas

Cada pasta reúne o subsistema, seus OpModes de teste em `Tests/` e seus ajustes
em `Tunning/`, quando existentes. Os subsistemas recebem `HardwareMap` e
`Telemetry`, executam `run(...)` a cada ciclo e publicam `sendTelemetry()`.
O TeleOp coordena entradas e chamadas dos subsistemas.

| Pasta | Responsabilidade |
| --- | --- |
| `Drive/` | Drive manual, field-centric, hold e heading pelo centro do HIVE |
| `Limelight/` | Câmera, validação de frames, mira e distância 3D ao centro das AprilTags |
| `FlywheelAdvanced/` | Motores, consulta da tabela e estabilização da RPM |
| `Intake/` | Coleta e ejeção |
| `Transfer/` | Alimentação dos elementos |
| `Trigger/` | Abertura do disparo |
| `DedinhoNervoso/` | Retenção/liberação pelo servo |

## TeleOp principal

`TeleOpMain` usa o `Follower` criado por `pedro.Constants`, com
Pinpoint e mecanum. A orientação ao apertar Play define a frente do campo.
O stick esquerdo controla a translação field-centric, o direito controla o giro
e o gatilho esquerdo aplica modo precisão. Ao parar e soltar os sticks, o
Pedro mantém a pose. Right bumper ativa a mira com translação livre; right
trigger aciona transfer/trigger. Sem visão válida, o giro retorna ao piloto.

Depois de atualizar a Limelight, o TeleOp chama:

```java
flywheel.runByDistance(limelight.getDistanceMeters());
```

A mira recebe `limelight.getObservation()`. No modo manual/teste, a flywheel
recebe uma RPM diretamente por `flywheel.run(rpm)`.

## Quatro classes principais

| Classe | Papel |
| --- | --- |
| `Limelight` | Câmera, configurações, centro 3D, distância ao centro e filtragem |
| `AprilTagObservation` | Dados imutáveis de heading publicados para o drive |
| `Flywheel` | Configurações, motores, RPM manual/automática, banda morta e telemetria |
| `FlywheelRpmTable` | Tabela imutável de pares medidos, validação e interpolação |

O TeleOp de calibração mantém seus próprios controles e passos de RPM.
Os ajustes de kS/kV/kP e os OpModes de teste ficam nas suas pastas.

## Limelight

As constantes ficam no início de `Limelight.java`: dispositivo, pipeline,
montagem, polling, idade máxima, janela e IDs. No `TeleOpMain`, D-pad cima
seleciona azul e baixo, vermelho. Esquerda seleciona a CELL da plateia (azul
38–41 / vermelho 34–37); direita seleciona a oposta (azul 42–45 / vermelho
30–33). Inicial: azul 42–45. A telemetria mostra aliança, CELL e IDs.
O método privado `estimateCenter()` projeta o centro do cluster pelas poses
das tags, considerando rotação e deslocamentos
conhecidos. Combina as estimativas do centro ponderadas pela área das tags.
A observação contém estado da mira, ângulo, quantidade de tags e timestamp.

`estimateCenter()` também combina a coordenada Y e calcula a norma 3D do
centro da CELL para obter a distância. Usa as poses individuais do SDK FTC
12.0.0, disponíveis no OS 2026.0 com Full3D, sem depender de `tdist`, POI ou
tag principal. `getRawDistanceMeters()` expõe a leitura bruta e
`getDistanceMeters()` a filtrada, em metros. A telemetria mostra
`Distancia centro filtrada (cm)`. Tags fora da CELL selecionada são ignoradas.
Pares antigos medidos por `tdist`/POI precisam ser recalibrados nessa referência.

A janela circular interna mantém até 40 frames distintos em um segundo,
separados por pelo menos 25 ms. Descarta 25% dos menores e dos maiores e
calcula a média restante (20 centrais com 40 amostras). Só recalcula quando a
janela muda. Perda de leitura, troca de HIVE ou reinício do timestamp limpa
o histórico, inclusive ao trocar de aliança. Imagens antigas, congeladas, pipeline incorreta e desconexão são
rejeitadas. A decodificação e a estimativa do centro ocorrem uma vez por frame/HIVE.

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

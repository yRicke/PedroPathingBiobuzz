# Geometria da mira: abertura e saida do lancador

O calculo usa as poses individuais `getTargetPoseCameraSpace()` e aplica duas
transformacoes no Java: centro da tag -> abertura e camera -> saida das bolas.
O mesmo alvo fisico deve alimentar o heading e a distancia da calibracao.

## Abertura oficial

Fonte: `AprilTagGameDatabase.getBioBuzzCluster()` do SDK FTC **12.0.0**,
presente na dependencia `org.firstinspires.ftc:Vision:12.0.0` do projeto.
Os centros das quatro tags, em polegadas e relativos a origem do cluster, sao:

| Membro | X | Y | Z |
| --- | --- | --- | --- |
| Primeiro ID | -6.5 | 7.1874 | -5.622 |
| Primeiro ID + 1 | -2.75 | 7.1874 | -5.622 |
| Primeiro ID + 2 | 2.75 | 7.1874 | -5.622 |
| Primeiro ID + 3 | 6.5 | 7.1874 | -5.622 |

A origem do cluster e o ponto nominal de mira da abertura definido pela FIRST.
A documentacao oficial descreve esse ponto como aproximadamente o centro da
abertura; nao garante erro fisico zero. A CELL aparece na figura 9-11 e as
tags na figura 9-16 do [manual, secao 9](https://ftc-resources.firstinspires.org/ftc/game/manual-09).
As coordenadas acima foram conferidas novamente no SDK 12.0.0 em 03/11/2026.
O alvo nao e o centro do adesivo, nem a media das tags que estao visiveis.
O vetor da tag ate essa origem e o negativo da linha correspondente,
convertido para metros. `AimGeometry.openingFromTag()` gira esse vetor pela
orientacao da tag e soma a posicao da tag no espaco da camera. Isso inclui
as mudancas de altura e profundidade durante a inclinacao da CELL.

A orientacao da pose da Limelight usa `Rz(yaw) * Ry(pitch) * Rx(roll)` para
essa transformacao. Os angulos da pose sao lidos em radianos. Os nomes dos
campos sao os do `Pose3D` entregue pelo SDK; a montagem da camera e tratada
separadamente, depois da estimativa da abertura.

## Medidas da montagem

O referencial de mira tem origem no centro da saida das bolas. Os eixos
acompanham o heading horizontal do lancador: X direita, Y baixo, Z frente.
O plano X/Z e horizontal; o referencial nao acompanha a inclinacao vertical
do mecanismo de lancamento. A distancia resultante e 3D, incluindo a altura.

O construtor de `AimGeometry` recebe:

| Medida | Unidade e sinal |
| --- | --- |
| Posicao lateral da lente | cm; direita positiva, esquerda negativa |
| Altura da lente em relacao a saida | cm; acima positiva, abaixo negativa |
| Posicao longitudinal da lente | cm; a frente positiva, atras negativa |
| Pitch da camera | graus acima da horizontal positivos |
| Yaw da camera | graus para a direita do heading do lancador positivos |
| Roll da camera | graus no sentido horario; montagem invertida: 180 |

`fromCamera()` aplica primeiro roll, depois pitch e yaw, e por ultimo soma
a posicao da lente. Dessa forma a inversao e a inclinacao da camera sao
compensadas nos tres eixos, antes de calcular o erro horizontal. Nao se
aplica uma segunda inversao do sinal de `tx` depois dessa transformacao.

Use medidas reais da montagem. Zero representa alinhamento e coincidencia
fisica, nao uma medida desconhecida. Mudar a camera, a saida das bolas ou
o angulo do lancador exige conferir novamente a geometria e a calibracao.

Montagem informada por CAD e confirmada em **02/11/2026**, aplicada nas
constantes `CAMERA_*` de `Limelight.java`: direita **0 cm**, acima **12,9 cm**,
a frente **-10,691 cm** (atras), pitch **+49,265 graus**, yaw **0 graus** e
roll **180 graus** (`UPSIDE_DOWN = true`). As distancias permanecem as medidas
confirmadas do CAD; o desenho posterior serve como ilustracao da montagem.

O angulo de saida das bolas informado e **70 graus acima da horizontal**.
Esse valor fica registrado como condicao mecanica da calibracao distancia/RPM.
O referencial de mira continua horizontal, com origem na saida das bolas.
Uma futura analise da trajetoria pode usar esse angulo junto com a velocidade
real de saida e a altura do alvo. Atualmente a tabela vem dos disparos reais.

## Painel e calibracao

Mantenha a pipeline 8 configurada com AprilTags, tamanho real das tags,
os IDs 30-45 e Full 3D. Para este calculo nao e necessario criar POI,
ajustar crosshair nem configurar robotspace no painel: o codigo usa
exclusivamente as poses no espaco da camera e a montagem medida no Java.

Antes de cadastrar os novos pares:

1. Confira o sentido do giro com o robo parado e o shooter desligado.
2. Confira o mesmo ponto de mira cobrindo tags diferentes da CELL selecionada.
3. Confira a distancia 3D da saida ate a abertura, em posicoes conhecidas.
4. Recalibre os pares distancia/RPM usando essa nova referencia.
5. Valide repeticao dos disparos perto/longe e em posicoes laterais diferentes.

Os pares antigos medidos da camera ate o adesivo precisam ser substituidos;
nao devem ser interpretados como medidas da saida ate a abertura.

## Consenso das poses e estabilidade do giro

Cada ID fornece uma estimativa independente da mesma abertura. Antes de
combinar essas posicoes, `Limelight` procura o maior grupo que concorda em
ate **12 cm** (`MAX_OPENING_DISAGREEMENT_METERS`). Conta IDs distintos;
somente depois aplica os pesos pela area dentro do grupo. Uma tag grande
com pose 3D ruim nao pode arrastar tres estimativas consistentes. Sem maioria
entre varias tags, a leitura fica indisponivel. O TeleOp devolve o giro ao
piloto; o autonomo conserva o hold e suspende a alimentacao. Uma unica tag
valida continua utilizavel. Nao existe busca pelas coordenadas da HIVE.

O erro visual pertence ao instante da captura. `AprilTagObservation` agora
informa a idade na primeira leitura: captura + processamento + parsing +
staleness do resultado. Leituras com latencia total acima de **300 ms** sao
descartadas. `PedroDrive` interpola o heading de captura em um historico local,
inclusive ao atravessar +/-180 graus, antes de converter o erro em heading
global. Sem historico suficiente, usa a velocidade angular atual para estimar
esse heading passado. O timestamp da Limelight identifica frames; nao e
subtraido diretamente do relogio do Control Hub. Frames repetidos conservam
o mesmo alvo global. Esta compensacao trata o giro, nao o deslocamento lateral
do robo durante a latencia.

Na mira com translacao livre, o retorno do controlador de heading recebe
amortecimento: `potencia = controlador - AIM_TURN_DAMPING * omega`, com `omega`
medido pelo localizador em rad/s. Nao se deriva o `tx`, que contem ruido.
Os valores iniciais ficam em `PedroDrive`:

| Constante | Valor | Uso |
| --- | --- | --- |
| `AIM_TURN_DAMPING` | 0.18 | Freia o giro antes de cruzar o alvo |
| `AIM_MAX_TURN_POWER` | 0.35 | Limita a potencia de giro automatico com translacao livre |
| `AIM_SETTLED_SPEED_RAD_S` | 10 graus/s | Velocidade maxima para declarar alinhamento |

A tolerancia angular continua em 1 grau; a correcao comandada continua limitada
a 8 graus e a precisao escala tambem o giro. No autonomo em hold, X/Y e a
frenagem continuam sob o Foresight do Pedro; a referencia visual recebe a
mesma compensacao de latencia e verificacao de velocidade para alinhamento.

No RB do TeleOp, `VisionHeadingTarget` filtra o heading visual global uma vez
por frame novo, com ganho 0,25 e passo maximo de 1 grau. Ao alinhar dentro
de 1 grau e com giro baixo, a potencia automatica fica em zero. A correcao
retoma ao atingir 1,5 grau; um giro real rapido ainda recebe frenagem.
Isso evita transformar ruido visual e pequenas oscilacoes do giroscopio
em inversoes sucessivas de potencia. As leituras visuais continuam sendo
atualizadas nessa faixa para permitir acompanhar deslocamentos reais.

A telemetria mostra a idade da imagem, poses rejeitadas, erro atual e giro
medido. Os testes cobrem camera atrasada com inercia, frames repetidos,
passagem por +/-180 graus, poses conflitantes e a montagem invertida em
aproximacoes laterais perto/longe com tags ocultas. O modelo de teste nao
substitui a validacao dos ganhos no robo. Confira a mira parado e transladando,
com poucas e varias tags visiveis, antes de registrar a calibracao de RPM.

Fontes: [clusters da FIRST](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html),
[poses da Limelight](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming),
[referenciais da Limelight](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems).

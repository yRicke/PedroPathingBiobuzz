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

A origem do cluster corresponde aproximadamente ao centro da abertura.
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

## Painel e calibracao

Mantenha a pipeline 8 configurada com AprilTags, tamanho real das tags,
os IDs 30-45 e Full 3D. Para este calculo nao e necessario criar POI,
ajustar crosshair nem configurar robotspace no painel: o codigo usa
exclusivamente as poses no espaco da camera e a montagem medida no Java.

Depois que a geometria estiver integrada e configurada:

1. Confira o sentido do giro com o robo parado e o shooter desligado.
2. Confira o mesmo ponto de mira cobrindo tags diferentes da CELL selecionada.
3. Confira a distancia 3D da saida ate a abertura, em posicoes conhecidas.
4. Recalibre os pares distancia/RPM usando essa nova referencia.
5. Valide repeticao dos disparos perto/longe e em posicoes laterais diferentes.

Os pares antigos medidos da camera ate o adesivo precisam ser substituidos;
nao devem ser interpretados como medidas da saida ate a abertura.

Fontes: [clusters da FIRST](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html),
[poses da Limelight](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming),
[referenciais da Limelight](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems).

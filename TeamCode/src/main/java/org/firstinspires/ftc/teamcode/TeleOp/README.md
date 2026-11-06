# TeleOpMain

TeleOp principal unico: drive do Pedro com field-centric, precisao e hold
automatico, localizacao recebida do autonomo e mira somente pela Limelight.
O antigo TeleOpMainPedro foi incorporado ao TeleOpMain e removido da lista de OpModes.
O modo Flywheel Calibration - 3D continua separado para calibracao manual de RPM.
O **Servo Tuner Universal**, grupo **Tuning**, ajusta um servo posicional por vez,
sem depender dos outros subsistemas. Veja o [guia de ajuste de servos](../../../../../../../../../docs/servo-tuning.md).

## Referencial e coordenadas

As constantes no inicio de TeleOpMain.java usam X/Y em polegadas e heading
em radianos, positivo anti-horario. INITIAL_POSE e RESET_POSE comecam em
(8.4, 35.2, 0 graus). O heading precisa corresponder a orientacao fisica
adotada como zero no referencial utilizado para a alianca atual.

A mira recebe somente a observacao da Limelight. Nao ha coordenadas de HIVE
nem busca automatica baseada em X/Y. A selecao da CELL escolhe os IDs:

| CELL | Azul | Vermelha |
| --- | --- | --- |
| Esquerda / plateia | 38-41 | 34-37 |
| Direita / oposta | 42-45 | 30-33 |

Inicial: azul, direita/oposta. Comandos opostos simultaneos no D-pad
preservam a selecao daquele eixo.

## Controles

| Controle | Acao |
| --- | --- |
| Gamepad 1, stick esquerdo | Translacao field-centric, inclusive durante a mira |
| Gamepad 1, stick direito | Giro manual fora da mira ou sem alvo confirmado |
| Gamepad 1, RB segurado | Mira pela Limelight, com translacao livre |
| Gamepad 1, LT | Precisao |
| Gamepad 1, RT | Aciona transfer e trigger enquanto pressionado |
| Gamepad 1, A | Alterna intake |
| Gamepad 1, LB | Inverte intake e transfer enquanto pressionado |
| Gamepad 1, Y | Frente atual vira referencia dos sticks; preserva a pose |
| Gamepad 1, X | Aplica RESET_POSE e restaura a frente global dos sticks |
| Gamepad 2, D-pad esquerda/direita | CELL plateia/oposta e hive esquerdo/direito |
| Gamepad 2, D-pad cima/baixo | Alianca azul/vermelha |

X e Y atuam uma vez por pressionamento; X tem prioridade. Use X quando o
robo estiver fisicamente na pose conhecida. O hold e a mira anteriores sao
cancelados para impedir uma tentativa de voltar a outra coordenada. Y altera
somente a referencia dos sticks. A mira continua usando o erro visual da CELL.

## Mira Limelight

Enquanto RB estiver pressionado, **tres frames novos consecutivos** com alvo
valido da mesma CELL confirmam o ajuste visual de heading.
Um frame repetido nao conta novamente. A Limelight ja rejeita imagens
antigas, congeladas, de pipeline incorreta e sem pose 3D valida.

Sem alvo ou sem confirmacao, o giro volta imediatamente ao stick direito.
Com sticks neutros, o drive conserva a pose atual. A nova aquisicao exige
tres frames. Trocar CELL/alianca, soltar RB, relocalizar ou reiniciar o
timestamp da camera cancela a confirmacao anterior.

A telemetria Mira informa confirmacao, Limelight ou giro manual. O limite
de correcao continua em 8 graus, com
tolerancia de alinhamento de 1 grau e escala de precisao. O erro visual usa
o heading da captura para compensar o atraso da imagem. O giro recebe
amortecimento pela velocidade medida e potencia maxima de 0,35; alinhamento
exige tambem giro de no maximo 10 graus/s. Esses parametros ficam nas
constantes `AIM_*` de `PedroDrive`. A mira controla
somente o giro; o hold de X/Y volta quando RB e solto e os sticks estao neutros.

A referencia visual e filtrada uma vez por imagem nova, com ganho 0,25 e
passo maximo de 1 grau. Ao entrar na faixa central de 1 grau com giro baixo,
pequenas variacoes deixam de virar comandos alternados. A correcao retoma
ao atingir 1,5 grau; giro real rapido ainda recebe frenagem. Os parametros
ficam em `Drive/VisionHeadingTarget.java`. A telemetria mostra o erro bruto,
o filtrado e a potencia de giro comandada.

O disparo continua sendo comandado pelo piloto no RT. A selecao da CELL
adequada e manual. A RPM usa a distancia 3D da saida das bolas ate a abertura
medida pela Limelight, com a geometria da camera ja configurada. Os pares
ficam em Flywheel e limitam o alvo ao menor/maior BASE_RPM (2560-2680 RPM hoje).
A leitura usa media central, suavizacao e banda morta de 1 cm; falhas breves de
deteccao preservam a distancia por ate 200 ms. Sem pares ou leitura, usa BASE_RPM.

## Pose do autonomo

AutonomousSouthOutside, AutonomousSouthOutsideV2, AutoPath e ExampleAuto salvam a ultima pose medida
no Stop, inclusive em paths interrompidos. Um autonomo cancelado antes
do Play nao publica pose. O TeleOpMain consome a pose uma unica vez no INIT;
sem ela, usa INITIAL_POSE. A telemetria informa a origem e acompanha movimentos
durante INIT. O Play preserva a pose e o heading recebidos: 23 graus continuam
sendo 23 graus no referencial do campo, compensados nos sticks.

O armazenamento e em memoria. Reiniciar o Robot Controller perde a pose.
Entre o Stop do autonomo e o INIT do TeleOp o localizador nao esta sendo
atualizado; se houver reposicionamento, relocalize na pose conhecida com X.

Referencias: [Pedro no TeleOp](https://pedropathing.com/docs/pathing/guide/teleop-usage),
[geometria da mira](../../../../../../../../../docs/aim-geometry.md),
[calibracao da flywheel](../../../../../../../../../docs/flywheel-tdist-calibration.md).

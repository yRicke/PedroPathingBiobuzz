# TeleOpMain

TeleOp principal unico: drive do Pedro com field-centric, precisao e hold
automatico, localizacao recebida do autonomo e mira hibrida Pedro/Limelight.
O antigo TeleOpMainPedro foi incorporado ao TeleOpMain e removido da lista de OpModes.
O modo Flywheel Calibration - 3D continua separado para calibracao manual de RPM.

## Referencial e coordenadas

As constantes no inicio de TeleOpMain.java usam X/Y em polegadas e heading
em radianos, positivo anti-horario. INITIAL_POSE e RESET_POSE comecam em
(8.4, 35.2, 0 graus). O heading precisa corresponder a orientacao fisica
adotada como zero no referencial utilizado para a alianca atual.

RIGHT_HIVE = (57.8, 50) e LEFT_HIVE = (57.8, 90) sao coordenadas de busca
aproximada. Conforme a montagem de campo informada, os mesmos numeros sao
usados nas duas aliancas: mudar azul/vermelho altera os IDs das tags.
O inicio, o reset e os caminhos do autonomo devem usar esse mesmo referencial.

A selecao da CELL determina tambem a coordenada de busca, sem uma segunda
selecao independente de hive:

| CELL | Coordenada Pedro | Azul | Vermelha |
| --- | --- | --- | --- |
| Esquerda / plateia | (57.8, 90) | 38-41 | 34-37 |
| Direita / oposta | (57.8, 50) | 42-45 | 30-33 |

Inicial: azul, direita/oposta. Comandos opostos simultaneos no D-pad
preservam a selecao daquele eixo.

## Controles

| Controle | Acao |
| --- | --- |
| Gamepad 1, stick esquerdo | Translacao field-centric, inclusive durante a mira |
| Gamepad 1, stick direito | Giro manual fora da mira |
| Gamepad 1, RB segurado | Mira hibrida: Pedro procura; Limelight ajusta |
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
somente a pilotagem; as coordenadas e o heading da mira continuam no referencial do Pedro.

## Mira hibrida

Enquanto RB estiver pressionado, o Pedro calcula o rumo para a coordenada
de busca da CELL selecionada. Depois de **tres frames novos consecutivos**
com alvo valido da mesma CELL, a Limelight assume o ajuste fino de heading.
Um frame repetido nao conta novamente. A Limelight ja rejeita imagens
antigas, congeladas, de pipeline incorreta e sem pose 3D valida.

Uma perda breve conserva o ultimo heading global por ate **100 ms**,
mantendo a translacao livre. Persistindo a perda, volta a procurar pelas
coordenadas. A nova aquisicao exige tres frames. Se a visao retornar antes
desse prazo, retoma o ajuste fino diretamente. Trocar a CELL, alianca ou
coordenada cancela a confirmacao anterior; imagens de outro alvo sao ignoradas.
Reinicio do timestamp da camera tambem exige nova confirmacao.

A telemetria Mira informa Pedro, confirmacao de visao, Limelight ou
manutencao breve de heading. O limite de correcao continua em 8 graus, com
tolerancia de alinhamento de 1 grau e escala de precisao. A mira controla
somente o giro; o hold de X/Y volta quando RB e solto e os sticks estao neutros.

O disparo continua sendo comandado pelo piloto no RT. A selecao da CELL
adequada e manual. A RPM usa a distancia 3D da saida das bolas ate a abertura
medida pela Limelight, com a geometria da camera ja configurada. Os pares
precisam ser recalibrados nessa referencia; sem pares ou leitura, usa BASE_RPM.

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

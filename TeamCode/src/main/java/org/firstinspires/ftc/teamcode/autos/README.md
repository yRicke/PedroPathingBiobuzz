# AutonomousSouthOutsideV3

OpMode **AutonomousSouthOutsideV3**, no grupo **Autonomous**. Usa os 11
trechos do Pedro Visualizer, com selecao de alianca no INIT e uma mirada
breve pela Limelight antes de cada disparo, sem exigir confirmacao.

## Percurso e paradas

As poses estao em polegadas e graus:

| Etapa | Pose final (X, Y, heading) | Acao |
| --- | --- | --- |
| Inicio | (57.8, 9, 90) | Aquece por 1,5 s, mira e dispara por 2 s antes de sair |
| chegadinha_frente_1 | (57.8, 20, 90) | Continua para o proximo trecho sem hold intermediario |
| prep_coleta_jardim | (31, 13, 180) | Continua para o proximo trecho sem hold intermediario |
| coleta_jardim | (14, 13, 180) | Continua para o proximo trecho sem hold intermediario |
| lancamento_2 | (60, 145, 270) | Para, mira e dispara por 2 s |
| chegadinha_frente_2 | (60, 122, 270) | Continua ate a preparacao da flower sem hold intermediario |
| prep_coleta_flower | (50.6, 122, 90) | Para e confirma o tempo reservado para abrir o coletor |
| coleta_flower | (46.6, 130, 90) | Mantem hold por 2 s com intake ligado |
| pos_flower | (46.6, 120, 90) | Recolhe o coletor e espera 0,5 s |
| pre_lancamento_3 | (60, 122, 270) | Continua ate o lancamento sem hold intermediario |
| lancamento_3 | (60, 145, 270) | Para, mira e dispara por 2 s |
| estacionamento | (10, 120, 0) | Mantem hold e desliga os mecanismos |

Cada etapa tem sua propria pose, editavel independentemente. Lancamentos
2 e 3 mantem as mesmas coordenadas; a segunda chegadinha e pre_lancamento_3
tambem. Os controles das duas curvas e as interpolacoes de heading foram
preservados.

Os trechos de transito sao concatenados com Paths.path em tres grupos:
chegadinha_frente_1 ate lancamento_2, chegadinha_frente_2 ate
prep_coleta_flower e pre_lancamento_3 ate lancamento_3. Dentro desses
grupos, o proprio Pedro faz a transicao entre segmentos, sem esperar hold
e velocidade zero em cada ponto intermediario. O auto so espera a parada
ao final de cada grupo e nos trechos individuais coleta_flower, pos_flower
e estacionamento. A telemetria continua identificando cada um dos 11
trechos originais.

Os tres grupos de deslocamento usam brakeAggression = 1.05, um ajuste
experimental pequeno sobre o padrao 1.0. O modificador e temporario e
restaura o valor anterior ao terminar o grupo ou receber Stop. Coleta da
flower, retorno pos_flower e estacionamento usam a configuracao original.
As tolerancias de chegada e a confirmacao de parada permanecem iguais.
Valores maiores podem ganhar tempo, mas permitem mais ultrapassagem do
alvo; 1.05 nao significa 5% menos tempo. Compare tempo e erro final na pista;
se precisar de muita correcao, volte TRANSIT_BRAKE_AGGRESSION para 1.0.
Referencia: [documentacao oficial do Pedro](https://pedropathing.com/docs/pathing/reference/pathconstraints).

O flower collector recebe a posicao fechada diretamente no construtor,
inclusive antes do Play. Abre quando o Pedro entra no segmento
prep_coleta_flower dentro do grupo. O auto somente
inicia coleta_flower depois de chegar a preparacao e de transcorrerem pelo
menos 500 ms desde a abertura. A espera de coleta de 2 s comeca quando
o follower termina o trecho e confirma a parada. O coletor permanece
aberto durante o retorno a pos_flower e fecha antes de pre_lancamento_3.
FLOWER_SERVO_SETTLE_MS reserva 500 ms para cada movimento dos servos;
servos posicionais nao fornecem confirmacao fisica da chegada.

O primeiro lancamento seleciona a CELL oposta; os outros dois selecionam
a CELL plateia. Antes de cada disparo, tenta ajustar a mira por 500 ms
(AIM_TIME_MS), mantendo X/Y fixos. Em seguida, conserva o heading medido
e alimenta o lancador continuamente por 2 s, sem exigir confirmacao de
visao, alinhamento ou RPM. Sem imagem, ainda dispara; perder visao ou RPM
durante a janela nao interrompe a alimentacao. O aquecimento inicial de
1,5 s e a mirada de 0,5 s antecedem a janela de disparo.

A flywheel continua com RPM por distancia e o intake fica ligado durante
os trechos e as esperas. Ao concluir o estacionamento, desliga intake,
flywheel, transfer e trigger. No Stop, recolhe o coletor, encerra os
mecanismos e a Limelight e salva a ultima pose medida para o TeleOp.
Cancelar antes do Play nao publica pose.

## Validacao automatizada do V3

AutonomousSouthOutsideV3Test simula o follower real com sensores e
mecanismos substituidos por mocks e relogio controlado. Verifica os 11
trechos, coordenadas das etapas, curvas, transicoes com velocidade diferente
de zero e tres janelas de disparo de 2 s. Tambem cobre
parada de coleta, ordem e tempo dos servos, selecao de CELL por alianca,
disparo sem confirmacao, continuidade sem visao/RPM e encerramento. FlowerCollectTest
tambem cobre a inicializacao fechada sem comando intermediario de abertura.

# AutonomousSouthOutsideV2

Novo OpMode no grupo **Autonomous**, preservando os seis trechos enviados:

1. Aquecimento inicial em hold por 1,5 s.
2. path1: start -> point1; tangente. Ao chegar e parar, mira e dispara.
3. path1_2: path1Start -> path1; heading linear.
4. path3: path1 -> point3; heading constante de 180 graus.
5. path4: curva com os tres controles fornecidos; heading linear de 180 a
   270 graus. Ao chegar e parar, mira e dispara.
6. path5: point4 -> point5; tangente.
7. path6: point6Start -> point6; heading linear. Termina em hold com os
   mecanismos desligados.

As coordenadas e as interpolacoes dos trechos foram mantidas. Durante cada
parada de disparo, o heading pode mudar para centralizar a abertura; X/Y
continuam fixos na pose final do path. O proximo trecho usa a interpolacao
que foi fornecida.

## Mesmo sistema do TeleOpMain

O drive confirma tres frames novos da CELL e usa somente a Limelight para
definir a mira. Nao procura coordenadas da HIVE.
No autonomo, essa mira preserva o hold de X/Y em vez de liberar a translacao.
O primeiro disparo seleciona direita/oposta e o segundo, esquerda/plateia.
Sem visao confirmada, conserva a posicao e o ultimo heading visual (ou o
heading do fim do path, antes da primeira leitura). A perda de visao exige
nova confirmacao e interrompe a alimentacao imediatamente.

No INIT, o D-pad do gamepad 2 seleciona a alianca: cima azul, baixo vermelha.
Inicial: azul. A escolha fica fixa depois do Play. Azul usa 42-45 no primeiro
disparo e 38-41 no segundo; vermelha usa 30-33 e depois 34-37.

A flywheel chama runByDistance(limelight.getDistanceMeters()) em todos os
ciclos ativos, inclusive durante os paths e as esperas. Usa a mesma geometria
da camera e a mesma tabela do TeleOp. Sem os novos pares calibrados, usa a
RPM base atualmente configurada em Flywheel; nao ha RPM fixa exclusiva desse auto.
O intake fica ligado ate o fim do percurso.

## Liberacao dos disparos

Depois de chegar ao fim de path1/path4, aguarda:

- Visao confirmada e erro angular dentro da tolerancia de 1 grau do drive.
- Robo parado, na posicao e no heading de hold.
- Os dois motores dentro de 100 RPM do setpoint.
- Todas essas condicoes mantidas por 150 ms.

Se a prontidao nao ocorrer em 3 s, o disparo e ignorado e o proximo path
comeca. A telemetria informa esse resultado. Quando pronto, abre uma janela
de disparo de 3 s. Durante essa janela, perder visao, posicao, heading ou
RPM interrompe a alimentacao; a retomada exige estabilizar novamente por
150 ms. A janela continua contando durante essas interrupcoes.

As constantes SPIN_UP_MS, AIM_TIMEOUT_MS, READY_STABLE_MS, SHOOT_TIME_MS e
RPM_TOLERANCE ficam no inicio de AutonomousSouthOutsideV2.java.

Ao terminar o percurso, conserva o hold de path6 e desliga intake, flywheel,
transfer e trigger. No Stop, salva a ultima pose medida em OpModeStorage
para o TeleOpMain e encerra tambem a Limelight. Cancelar antes do Play nao
publica pose. O autonomo antigo AutonomousSouthOutside continua disponivel.

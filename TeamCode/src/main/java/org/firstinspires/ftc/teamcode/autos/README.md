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

O drive compartilha a logica hibrida do TeleOp: procura pelas coordenadas,
confirma tres frames novos da CELL e usa a Limelight para o ajuste fino.
No autonomo, essa mira preserva o hold de X/Y em vez de liberar a translacao.
O primeiro disparo seleciona direita/oposta e o segundo, esquerda/plateia.
As coordenadas de busca sao obtidas diretamente de TeleOpMain.targetForCell,
assim os dois modos compartilham os valores (57.8, 50) e (57.8, 90).

No INIT, o D-pad do gamepad 2 seleciona a alianca: cima azul, baixo vermelha.
Inicial: azul. A escolha fica fixa depois do Play. Azul usa 42-45 no primeiro
disparo e 38-41 no segundo; vermelha usa 30-33 e depois 34-37. Os numeros das
coordenadas permanecem iguais, no referencial definido para a alianca atual.

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

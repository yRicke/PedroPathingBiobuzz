# Ajuste de um servo por vez

Selecione **Servo Tuner Universal**, grupo **Tuning**, na Driver Station.
O código fica em `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TeleOp/ServoTuningTeleOp.java`.
Ele não instancia o `FlowerCollect`, o drive ou os outros subsistemas.

## Preparação e controles

1. Com o robô desligado, conecte o servo à porta desejada do Hub. Configure essa
   porta como **Servo** na Robot Configuration e dê um nome a ela. Pode manter
   `flowerCollect1`, `flowerCollect2` ou qualquer outro nome: não é necessário
   editar Java para trocar o dispositivo. O tuner lista os servos posicionais
   `ServoImplEx` dos Hubs REV; dispositivos configurados como `CRServo` não entram.
2. No primeiro teste do servo de cinco voltas, desacople o braço/mecanismo.
   Verifique também se o servo está programado no modo posicional.
3. Aperte **INIT**. O tuner desliga o PWM de todos os servos listados e não envia
   posições. Escolha o servo pelo nome e confira conexão e porta na telemetria.
4. Ajuste alvo, sentido e faixa PWM. Aperte **Play** e depois **A** para aplicar
   o alvo e ligar somente a saída selecionada. O alvo inicial é **0.500**; o
   primeiro acionamento pode movimentar várias voltas, conforme a posição física.
5. Ajuste com toques no D-pad, observe o mecanismo e anote os valores encontrados.
   **B** desliga o PWM imediatamente; o servo pode deixar de sustentar a carga.
   Pare o teste se ele ficar forçando um batente. O **Stop** desliga os PWMs listados.

| Controle 1 | Ação |
| --- | --- |
| A, depois do Play | Aplica o alvo e liga o PWM do servo selecionado |
| B | Desliga o PWM; tem prioridade sobre A e os ajustes |
| D-pad cima / baixo | Aumenta / diminui a posição alvo, limitada entre 0 e 1 |
| D-pad esquerda / direita | Diminui / aumenta o passo: 0.001, 0.005, 0.010, 0.050, 0.100 |
| LB / RB, com PWM desligado | Seleciona o servo anterior / próximo, em ordem de nome |
| X, com PWM desligado | Alterna padrão SDK (600–2400 µs) e goBILDA 5-turn (500–2500 µs) |
| Y, com PWM desligado | Alterna FORWARD / REVERSE |

Cada botão atua uma vez por toque; segurar não repete. Os ajustes também podem
ser preparados durante o INIT, mas A só funciona depois do Play. Solte A entre
o INIT e o Play. Trocar de servo reinicia o alvo em 0.500, o sentido em FORWARD
e a faixa no padrão SDK. Sentido, perfil e seleção ficam bloqueados enquanto
o PWM está ligado. Alterar configuração junto com A exige soltar e apertar A
novamente. Com PWM desligado, editar o alvo não envia posição ao servo.

A telemetria mostra o **alvo**, o passo, o sentido e o pulso correspondente em
microssegundos; não há sensor de posição lido por esse programa. O tuner não
grava a calibração. Anote **nome/porta, posição, sentido e faixa PWM**. Use os
mesmos sentido e faixa PWM no código definitivo para reproduzir a posição.

## goBILDA Torque de cinco voltas

Para o modelo **2000-0025-0502**, a
[ficha da goBILDA](https://www.gobilda.com/2000-series-5-turn-dual-mode-servo-25-2-torque/)
informa **500–2500 µs, 50 Hz e até 1800°** no modo posicional. O intervalo
**0 a 1 é correto** no SDK FTC: é a fração da faixa PWM, não uma quantidade de
voltas. Com perfil 500–2500 µs e FORWARD:

| Posição | Pulso | Deslocamento nominal a partir do extremo de 500 µs |
| --- | --- | --- |
| 0.000 | 500 µs | 0° |
| 0.500 | 1500 µs | 900° (2,5 voltas) |
| 1.000 | 2500 µs | 1800° (5 voltas) |

O zero não é uma referência da montagem do braço. Uma diferença de **0.010**
corresponde nominalmente a **18°**, e **0.100** a **180°** nesse perfil. Comece
com passo pequeno. O passo 0.001 corresponde a 2 µs, abaixo da banda morta de
4 µs informada pelo fabricante; alguns toques pequenos podem acumular antes
de aparecer movimento. A faixa padrão SDK é mais estreita e não cobre as
cinco voltas completas; isso, por si só, não explica ausência total de movimento.
REVERSE espelha o pulso: o alvo 0 vira o extremo superior e 1 o inferior.

Esse servo também pode estar programado em **rotação contínua**: nesse modo o
PWM controla velocidade/sentido, não posição. Trocar `Servo.Direction` ou usar
`Servo` em vez de `CRServo` em Java **não reprograma o modo interno** do servo.
Para voltar ao modo posicional, conecte-o ao Servo Programmer, coloque a chave
em **S** e dê um **toque curto em P**; o LED P pisca três vezes ao concluir,
conforme as [instruções oficiais do programador](https://www.gobilda.com/servo-programmer-for-2000-series-dual-mode-servo/).

## Quando ele só força para um lado

O tuner anterior chamava o construtor de `FlowerCollect` já no INIT, enviando
0 lógico aos dois servos, um FORWARD e outro REVERSE. Isso significa extremos
opostos da faixa e pode prender o mecanismo em um batente. O novo tuner
permite testar o servo isolado e com acionamento explícito. Isso não confirma
qual foi a falha física do teste anterior.

Com o mecanismo desacoplado, confirme o modo posicional e teste alvos próximos,
como 0.490, 0.500 e 0.510. Observe se a saída realmente chega a posições diferentes.
Se continuar forçando sem girar, desligue o PWM e confira nome/porta do Hub,
orientação do conector, cabo/extensão e alimentação. Teste o servo em outra
porta e no programador para separar problema no Hub/cabo de problema no servo.
O valor de `getPosition()` também é só o último comando, não comprova movimento,
como explica a [API Servo do FTC](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Servo.html).

Para o `FlowerCollect`, anote aberto/fechado de cada lado separadamente usando
FORWARD no primeiro e REVERSE no segundo. Seus valores 0 e 1 ainda são
provisórios. Se usar o perfil 500–2500 µs, o código definitivo também precisa
configurar essa faixa com `ServoImplEx.setPwmRange(new PwmControl.PwmRange(500, 2500))`
antes de aplicar os alvos. O tuner não altera essas constantes automaticamente.

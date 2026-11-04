# Ajuste de um servo por vez

Selecione **Servo Tuner Universal**, grupo **Tuning**, na Driver Station.
O código fica em `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TeleOp/ServoTuningTeleOp.java`.
Ele não instancia o `FlowerCollect`, o drive ou os outros subsistemas.

## Roteiro para a equipe: conexão e instalação

O **Control Hub/Expansion Hub** recebe o cabo do servo; o **Driver Hub** roda
o aplicativo **FTC Driver Station** e recebe o gamepad USB.

1. Separem robô com bateria carregada, Driver Hub, controle USB e, se o código
   ainda não estiver no robô, notebook com o projeto **PedroPathingBiobuzz**.
   Deixem o braço do servo desacoplado no primeiro teste. Se dois servos movem
   a mesma peça, desacoplem a transmissão entre eles para testar um por vez.
2. Desliguem o robô antes de conectar os cabos. Usem uma porta marcada **SERVO**
   do Control Hub ou Expansion Hub, anotando qual Hub e o número da porta.
   Alinhem o fio preto/marrom ao **GND/-**, vermelho ao positivo e sinal ao **S**,
   seguindo as marcações do Hub. Não usem portas de motor, digital ou I2C.
   A [documentação FTC de conexão](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/connecting_devices/connecting_servo/connecting-servo.html)
   mostra o conector de três fios e a orientação do GND.
3. Liguem o robô. Se precisarem instalar o programa, conectem o notebook à porta
   USB-C do Control Hub com cabo de dados. No Android Studio, abram
   **PedroPathingBiobuzz**, selecionem **TeamCode** e o dispositivo **REV Robotics
   Control Hub** na barra superior e apertem **Run**. Aguardem a instalação
   terminar sem erro. O programa precisa estar instalado no Control Hub.
   A [documentação Android Studio da FTC](https://ftc-docs-cdn.ftclive.org/booklets/en/android_studios.pdf)
   descreve esse procedimento.
4. Abram **FTC Driver Station** no Driver Hub. Se não conectar ao robô, entrem
   em **⋮ > Settings > Pair with Robot Controller > Wi-Fi Settings**, escolham
   a rede do Control Hub da equipe e informem a senha da equipe. Voltem à tela
   principal e confiram conexão, ping e tensão da bateria. Sigam o
   [pareamento oficial da REV](https://docs.revrobotics.com/duo-control/menu/control-hub-gs/driver-station-pairing-to-control-hub)
   se necessário.
5. Conectem o gamepad por USB ao Driver Hub. No Logitech F310 ou controle Xbox
   compatível, pressionem **Start + A** para registrá-lo como controle 1 e
   soltem os botões. Confiram o ícone do controle 1 na Driver Station. No modelo
   REV compatível com PS4, o equivalente é **Options + X (cruz)**.
   Veja a [orientação de conexão de gamepads da REV](https://docs.revrobotics.com/rev-professional-development-technical-path/gs/setup/connecting-devices).

## Configurar a porta pela Driver Station

1. Com os OpModes parados, abram **⋮ > Configure Robot** no Driver Hub.
2. Editem a configuração do robô que já está em uso. Se for uma bancada sem
   configuração, usem **New** e aguardem a busca dos Hubs. Não recriem a
   configuração inteira do robô para ajustar um servo.
3. Abram o **Control Hub Portal/Expansion Hub Portal**, quando essa tela aparecer,
   e selecionem o Hub onde o cabo está conectado. Entrem em **Servos**.
4. Na linha do número da porta física escolhida, selecionem o tipo **Servo**.
   O tuner de posição não lista portas configuradas como **Continuous Rotation
   Servo/CRServo**. Essa escolha no aplicativo não muda o modo interno do servo.
5. Nomeiem o primeiro lado **flowerCollect1** e o segundo **flowerCollect2**,
   respeitando maiúsculas/minúsculas. Para teste genérico, qualquer nome funciona;
   para o FlowerCollect definitivo, mantenham esses dois nomes. Nomes devem ser
   únicos. Se já estiverem configurados nas portas corretas, preservem-nos.
6. Voltem com **Done**, usem **Save**, confirmem o nome da configuração e retornem
   à tela principal. Confiram que ela aparece como configuração ativa. No tuner,
   façam um novo **INIT** depois de alterar a configuração.

O [tutorial FTC de configuração](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/getting_started/getting-started.html)
mostra a seleção dos Hubs; a [página de servos](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/configuring_servo/configuring-servo.html)
mostra tipo, porta e nome. A [página de salvamento](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/saving_config/saving-config.html)
explica como salvar e confirmar a configuração ativa.

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

## Encontrar aberto e fechado do FlowerCollect

Este roteiro usa o perfil **goBILDA 5-turn, 500–2500 µs** para o modelo de cinco
voltas. A programação definitiva precisará usar a mesma faixa antes de aplicar
os valores medidos; o `FlowerCollect` atual ainda não configura essa faixa
explicitamente. A equipe deve devolver os valores para serem aplicados no código.

1. Na lista de **TeleOp** da Driver Station, escolham **Servo Tuner Universal**,
   grupo **Tuning**. Apertem **INIT**. Confiram **PWM DESLIGADO**. Nesse instante
   o tuner desliga todos os servos posicionais listados, inclusive os demais.
2. Com **LB/RB**, escolham **flowerCollect1**. Confiram o nome e a porta na tela.
   Apertem **X** até mostrar **goBILDA 5-turn / 500 a 2500 µs**. Mantenham
   **FORWARD**. Para **flowerCollect2**, usem **Y** até mostrar **REVERSE**.
3. Com o servo desacoplado, mantenham alvo **0.500**. Soltem os botões,
   apertem **Play** na Driver Station e então deem um toque em **A** no gamepad.
   Aguardem o eixo parar. Esse primeiro movimento pode percorrer várias voltas.
4. Para confirmar funcionamento, selecionem passo **0.005** com D-pad direita.
   Deem um toque em cima: o alvo vai de 0.500 para 0.505. Deem um toque em baixo:
   volta a 0.500. O eixo deve ir a duas posições e parar em cada uma. Se ficar
   girando sem parar, confiram o modo interno; se só forçar, desliguem com B.
5. Antes de reacoplar, retornem a **0.500**, aguardem parar, apertem **B**, **Stop**
   e desliguem o robô. Montem o braço com o mecanismo em uma posição intermediária
   do curso útil, sem girar o eixo para alinhar. Mantenham a transmissão do outro
   servo desacoplada durante o teste individual. Se a montagem exigir girar o
   eixo ou não permitir essa referência, alinhem a montagem antes de prosseguir.
6. Liguem novamente e repitam seleção, perfil e sentido no tuner. Confirmem
   **0.500**, apertem **Play** e depois **A**, observando o mecanismo. A posição
   física precisa continuar compatível com esse alvo.
7. Usem o passo **0.001** e pequenos toques para levar o mecanismo à abertura
   desejada. Observem o sentido real: aumentar o número pode abrir ou fechar,
   conforme o lado e a montagem. Anotem a posição de **aberto** com três casas.
8. Ajustem aos poucos até a posição **fechada útil**, sem apertar contra batente
   nem deixar o servo forçando. Anotem a posição de **fechado**. Não pulem para
   0 ou 1 para procurar limites: num servo de cinco voltas isso pode ser um
   deslocamento muito grande.
9. Vão de aberto a fechado e retornem algumas vezes pelos ajustes, usando passos
   pequenos perto dos limites. Confirmem que o braço volta ao lugar e que o
   mecanismo não fica tensionado. A telemetria mostra o comando, por isso é
   necessário observar o movimento real. Se B for usado, editem o alvo com PWM
   desligado e apertem A novamente para aplicá-lo, evitando saltos grandes.
10. Apertem **B** antes de trocar de servo com **LB/RB**. A troca reinicia
    **posição 0.500, FORWARD e perfil padrão SDK**: refaçam o perfil 500–2500 µs
    e, no segundo lado, **REVERSE**. Repitam a calibração individual e anotem
    os dois valores do outro lado. Conexões mecânicas só devem ser alteradas
    depois de Stop e com o robô desligado.

Se a mecânica mantiver ambos os servos rigidamente acoplados, preparem a
calibração individual desacoplando-os. Deixem a validação conjunta para depois
de aplicar os quatro valores e a faixa PWM no código. Este tuner aciona uma
saída por vez e não sincroniza um par mecanicamente acoplado.

### Resultado para devolver à programação

| Campo | Servo 1 | Servo 2 |
| --- | --- | --- |
| Nome | flowerCollect1 | flowerCollect2 |
| Hub e porta | preencher | preencher |
| Sentido | FORWARD | REVERSE |
| Faixa PWM | 500–2500 µs | 500–2500 µs |
| Aberto | preencher com 3 casas | preencher com 3 casas |
| Fechado | preencher com 3 casas | preencher com 3 casas |
| Repetiu sem forçar? | sim/não | sim/não |

Enviem essa tabela, uma foto das conexões e, se possível, um vídeo curto do
movimento. O tuner não salva os valores e sair do programa não atualiza as
constantes. O subsistema atual ainda tem 0/1 provisórios e comanda aberto no
construtor; usem o tuner para a calibração e aguardem a atualização do código
antes de validar o par montado com o subsistema.

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

- **O tuner não aparece na lista:** confirmar a instalação do projeto atualizado
  no Control Hub; reabrir a conexão da Driver Station após a instalação.
- **Nenhum servo listado:** conferir tipo Servo, Hub/porta, configuração ativa,
  salvamento e novo INIT. A lista representa a configuração, não detecta se o
  cabo do servo está fisicamente conectado.
- **Nenhum botão funciona:** conferir gamepad registrado como controle 1,
  programa em Play e PWM ligado por A. LB/RB/X/Y exigem PWM desligado.
- **O número muda com PWM desligado:** isso apenas prepara o alvo. A é necessário
  para aplicá-lo. Com PWM ligado, os ajustes de posição já são enviados.
- **Só pequenos toques não produzem movimento visível:** considerar a banda
  morta descrita acima; testar 0.005 com o mecanismo desacoplado.
- **Não para de girar:** B, Stop e conferir modo posicional no programador.
- **Está forçando ou esquentando:** B e Stop; desligar o robô antes de inspecionar
  montagem, cabos e alimentação. Não insistir contra um batente.

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

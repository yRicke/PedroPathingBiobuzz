# TeleOpMainPedro

Variante experimental do `TeleOpMain`. O modo original continua disponivel.
Preserva intake, flywheel por distancia da Limelight, transfer, trigger,
drive field-centric, precisao e hold automatico ao soltar os controles.

As constantes no inicio de `TeleOpMainPedro.java` usam o referencial global
do Pedro: X/Y em polegadas, heading em radianos (positivo anti-horario).
`INITIAL_POSE` define a pose ao iniciar; `RESET_POSE` e a pose conhecida usada
para corrigir a localizacao. Ambas comecam em `(8.4, 35.2, 0 graus)`.
O heading tambem precisa corresponder a orientacao fisica do robo.
`RIGHT_HIVE` comeca em `(57.8, 50)` e `LEFT_HIVE` em `(57.8, 90)`.
Essas coordenadas sao absolutas; nao sao espelhadas pela selecao de alianca.

| Controle | Acao |
| --- | --- |
| Gamepad 1, Y | Define a frente atual como frente dos sticks, preservando a pose global |
| Gamepad 1, X | Aplica `RESET_POSE` e restaura a frente global dos sticks |
| Gamepad 1, right bumper | Mira no hive selecionado pelas coordenadas do Pedro, com translacao livre |
| Gamepad 1, left trigger | Modo de precisao |
| Gamepad 1, right trigger | Dispara |
| Gamepad 1, A / left bumper | Intake / reversao do intake e transfer |
| Gamepad 2, D-pad esquerda/direita | Ativa somente o hive esquerdo/direito (inicial: direito); tambem seleciona CELL plateia/oposta na Limelight |
| Gamepad 2, D-pad cima/baixo | Seleciona alianca azul/vermelha para a distancia da Limelight |

Os resets ocorrem uma vez por pressionamento; X tem prioridade sobre Y.
Use X somente quando o robo estiver fisicamente na pose de reset. O hold
antigo e cancelado para impedir que a correcao de coordenadas provoque uma
tentativa de voltar a posicao anterior. Y pode ser usado em qualquer pose,
pois altera apenas a referencia de pilotagem.

A telemetria mostra X/Y, heading global e heading relativo dos controles.
Depois de Y, o heading global continua sendo a orientacao verdadeira no campo;
somente o heading dos controles passa a zero.

A mira calcula `atan2(alvoY - roboY, alvoX - roboX)` continuamente, usa o menor
erro angular e controla somente o giro com o heading lock do Pedro. Preserva
a tolerancia de 1 grau, limite de correcao de 8 graus e escala de precisao da
mira original. Ao soltar o bumper, o giro volta ao piloto e o hold automatico
volta a atuar com os sticks neutros. Se alvo e robo coincidirem, o giro fica
manual. Pressionar esquerda e direita juntos preserva a selecao anterior.

A mira funciona sem a Limelight; o calculo de RPM por distancia ainda usa
a camera, como no `TeleOpMain`. Os seletores de alianca e CELL da camera
continuam afetando essa distancia. Confira a correspondencia entre a CELL
selecionada e o hive fisico ao testar os disparos.

Referencia: [uso do Pedro no TeleOp](https://pedropathing.com/docs/pathing/guide/teleop-usage).

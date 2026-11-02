# TeleOpMainPedro

Variante experimental do `TeleOpMain`. O modo original continua disponivel.
Preserva intake, flywheel por distancia da Limelight, transfer, trigger,
drive field-centric, precisao e hold automatico ao soltar os controles.

As constantes no inicio de `TeleOpMainPedro.java` usam o referencial global
do Pedro: X/Y em polegadas, heading em radianos (positivo anti-horario).
`INITIAL_POSE` define a pose ao iniciar; `RESET_POSE` e a pose conhecida usada
para corrigir a localizacao. Ambas comecam em `(8.4, 35.2, 0 graus)`.
O heading tambem precisa corresponder a orientacao fisica do robo.

| Controle | Acao |
| --- | --- |
| Gamepad 1, Y | Define a frente atual como frente dos sticks, preservando a pose global |
| Gamepad 1, X | Aplica `RESET_POSE` e restaura a frente global dos sticks |
| Gamepad 1, right bumper | Mira com a Limelight e permite translacao livre |
| Gamepad 1, left trigger | Modo de precisao |
| Gamepad 1, right trigger | Dispara |
| Gamepad 1, A / left bumper | Intake / reversao do intake e transfer |
| Gamepad 2, D-pad esquerda/direita | Seleciona CELL plateia/oposta |
| Gamepad 2, D-pad cima/baixo | Seleciona alianca azul/vermelha |

Os resets ocorrem uma vez por pressionamento; X tem prioridade sobre Y.
Use X somente quando o robo estiver fisicamente na pose de reset. O hold
antigo e cancelado para impedir que a correcao de coordenadas provoque uma
tentativa de voltar a posicao anterior. Y pode ser usado em qualquer pose,
pois altera apenas a referencia de pilotagem.

A telemetria mostra X/Y, heading global e heading relativo dos controles.
Depois de Y, o heading global continua sendo a orientacao verdadeira no campo;
somente o heading dos controles passa a zero.

Referencia: [uso do Pedro no TeleOp](https://pedropathing.com/docs/pathing/guide/teleop-usage).

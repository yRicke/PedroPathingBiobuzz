# Mira de AprilTag com Limelight e Pedro

## Objetivo aprovado

No `TeleOpMainTransferTest`, D-pad esquerdo seleciona a AprilTag 39 e direito
seleciona a 43. Enquanto o `right_trigger` do gamepad 1 estiver pressionado,
o robô mira e dispara juntos. O Pedro mantém X/Y e altera somente o heading
para manter a tag selecionada na linha vertical central da câmera. Soltar o
gatilho devolve o drive field-centric ao piloto.

## Componentes e configuração

- `Limelight.run(boolean dpadLeft, boolean dpadRight)` mantém a seleção,
  lê a câmera e publica uma observação imutável. Seleção inicial: 43.
  Pressionar as duas direções simultaneamente preserva a seleção anterior.
- O hardware chama-se `limelight`. O índice da pipeline AprilTag começa em 0
  e é configurável no TeleOp, assim como a montagem normal/invertida.
- A inversão física corrige o sinal horizontal da observação. Usa-se o ângulo
  sem deslocamento do crosshair (`getTargetXDegreesNoCrosshair`).
- `AprilTagObservation` distingue câmera indisponível, imagem recente sem o
  alvo selecionado e alvo visível. Só o resultado do ID selecionado é usado.
- `PedroDrive.aim(AprilTagObservation)` captura X/Y na entrada, mantém esse
  ponto durante a mira e atualiza o heading usando o `Follower.hold()`.

## Controle e transições

- Imagens com idade superior a 150 ms, timestamps inválidos, pipeline errada
  ou câmera desconectada não autorizam giro de busca nem correções visuais.
  Um timestamp que não avança por mais de 150 ms também indica imagem congelada,
  mesmo que o polling continue recebendo respostas HTTP recentes.
- Sem a tag selecionada em uma imagem recente, procura devagar: alvo de
  heading 2 graus à frente da direção de busca, inicialmente horária.
  Depois de ver a tag, a busca segue a direção do último erro horizontal.
- Com a tag visível, converte graus da imagem para heading em radianos,
  com correção máxima de 8 graus por imagem e tolerância de 1 grau.
- Cada timestamp da câmera só atualiza o alvo uma vez; imagens repetidas
  não somam a mesma correção ao heading corrente.
- Ao perder dados recentes, congela o heading corrente e mantém o X/Y
  capturado. Ao soltar o gatilho, cancela o alvo anterior antes do drive
  manual para impedir que uma rotação de busca continue após a soltura.
- Transfer e trigger mantêm o comportamento de disparo atual; não há bloqueio
  de disparo por ausência de tag ou câmera, conforme escolha do usuário.
- O follower é atualizado uma única vez por iteração e continua sendo o
  único proprietário dos motores do drivetrain.

## Validação

Testes de seleção, orientação invertida, ID incorreto, imagem antiga,
pipeline errada e desconexão usam doubles apenas na fronteira do SDK/hardware.
Testes do drive usam o Follower e Foresight reais com localização/drivetrain
simulados: X/Y fixos, sinais do heading, busca, tolerância, repetição de frame,
troca de alvo e soltura do gatilho. Rodar a suíte debug/release e montar APK.
O usuário valida os sinais e a resposta física no robô antes do uso em campo.

## Referências oficiais

- https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
- https://docs.limelightvision.io/docs/docs-limelight/getting-started/pipelines
- https://pedropathing.com/docs/pathing/guide/teleop-usage
- SDK FTC Hardware 12.0.0 e Pedro Pathing core/revhub 3.0.1 usados no projeto.

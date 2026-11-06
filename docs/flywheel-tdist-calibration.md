# Calibrar a flywheel com POLLEN e distância 3D até a abertura da CELL

## Primeiro teste

1. Abra o TeleOp **Flywheel Calibration - 3D**, grupo **Flywheel**. Ele não
   aciona o drivetrain; coloque o robô na posição de teste.
2. Confira a pipeline 8, aliança e CELL mostradas (azul 42–45 inicialmente).
   No gamepad 2, D-pad cima/baixo seleciona azul/vermelha e esquerda/direita
   seleciona CELL plateia/oposta. Aguarde `Distancia abertura (cm)` válida.
3. A RPM desejada começa em `Flywheel.BASE_RPM` (**2570** atualmente), com
   todos os mecanismos desligados. No gamepad 1, B alterna a flywheel.
   A alterna o intake, LB inverte intake/transfer e RT dispara, como no
   TeleOpMain. Ajuste a RPM e o passo pelo D-pad.
4. Aguarde a RPM medida se aproximar da desejada e a distância estabilizar
   depois de mover o robô (janela de um segundo mais suavização de 250 ms).
5. Com a flywheel na RPM desejada, segure RT para acionar transfer e trigger.
   Soltar RT desliga a alimentação e fecha o trigger. A liga/desliga o intake;
   LB inverte intake e transfer enquanto pressionado e tem prioridade no transfer.
6. Quando encontrar um bom ajuste, anote à mão a distância e a RPM desejada.
   Repita em outras posições. A tela permanece ao vivo, sem captura de pares.

| Controle (gamepad 1) | Ação |
| --- | --- |
| D-pad cima / baixo | Aumenta / diminui RPM desejada, um passo por clique |
| D-pad esquerda / direita | Diminui / aumenta o passo: 0,1; 1; 10; 100; 1000 RPM |
| A | Liga / desliga intake, como no TeleOpMain |
| LB | Inverte intake e transfer enquanto pressionado |
| RT | Aciona transfer e abre trigger enquanto pressionado |
| B | Liga / desliga flywheel mantendo o ajuste de RPM |

No gamepad 2, D-pad cima/baixo seleciona aliança azul/vermelha e
esquerda/direita seleciona CELL plateia/oposta, como no TeleOp principal.

A tela mostra somente distância, RPM desejada/medida, passo, estado dos
quatro mecanismos e alvo selecionado. A distância exibida já é filtrada e
usa a referência 3D da saída das bolas até a abertura da CELL.

O passo inicial é 10 RPM. Segurar D-pad/A/B não repete o comando a cada loop.
RT e LB são momentâneos, como no TeleOpMain.
A calibração permite de zero a `Flywheel.MAX_RPM` (**6000 RPM**).
Somente o automático por distância fica entre `AUTO_MIN_RPM` e `AUTO_MAX_RPM`
(**2560–2680 RPM**), derivados do menor e maior `BASE_RPM*`. B desliga a flywheel
enviando zero. Essas constantes ficam em `Flywheel`; `PIPELINE_INDEX` e
`UPSIDE_DOWN` ficam em `Limelight`. Ao parar o OpMode, a flywheel, o
transfer e o intake são desligados, o trigger é fechado e a câmera é parada.

## Configuração da Limelight

Siga o [guia do painel e das AprilTags BIOBUZZ](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Limelight/README.md),
com pipeline 8, família 36h11, tamanho nominal 82,55 mm, filtros dos 16 IDs,
e Full3D. **Limelight OS 2026.0 é suficiente**; não é necessário atualizar
para 2026.1, configurar POIs ou fornecer `tdist` no JSON.

O Java calcula a distância **3D da saída das bolas até a abertura da CELL
selecionada** a partir de `getTargetPoseCameraSpace()`, da geometria oficial
do SDK FTC 12.0.0 e da montagem da câmera. Inclui altura, profundidade e
deslocamento da lente; a mira usa essa mesma abertura. Tags de outras CELLS
são ignoradas. A tabela recebe a leitura filtrada convertida para centímetros.

A montagem configurada está centralizada, 12,9 cm acima e 10,691 cm atrás
da saída das bolas, com pitch de +49,265 graus, yaw zero e roll 180.
Confira o sentido de giro antes de anotar pares. Os valores
ficam nas constantes `CAMERA_*` e `UPSIDE_DOWN` de `Limelight.java`.
Não é necessário configurar a montagem, POI ou crosshair no painel.

O ângulo de saída das bolas informado é 70 graus acima da horizontal.
Faça a calibração com esse ajuste mecânico; a tabela registra o comportamento
dos disparos reais. Alterar esse ângulo exige revalidar os pares.

Os pares atuais são 182 cm / 2560 RPM, 185 cm / 2570 RPM e 205 cm / 2680 RPM.
Refaça pares obtidos por `tdist`/POI ou da câmera até o centro do adesivo.
Calibração e modo automático usam a mesma referência calculada em Java, sem
depender da tag principal. Veja [a geometria da mira](aim-geometry.md).

## Aplicar os pares

Edite `subsystems/FlywheelAdvanced/Flywheel.java`:

| Par | Distância saída → abertura filtrada (cm) | RPM desejada anotada |
| --- | --- | --- |
| 1 | `BASE_DISTANCE_CM` | `BASE_RPM` |
| 2 | `BASE_DISTANCE_2_CM` | `BASE_RPM_2` |
| 3 | `BASE_DISTANCE_3_CM` | `BASE_RPM_3` |

Distância zero desativa o par até a medição. Preencha distância e RPM de cada
par validado, sem duplicar distâncias. A tabela aceita qualquer ordem e copia
os valores na inicialização. Para mais pontos, adicione pares em
`Flywheel.createRpmTable()` e gere o programa novamente.

- Sem pares: usa `BASE_RPM` (2570 atualmente).
- Um par: mantém a RPM desse par; não existe curva medida ainda.
- Dois ou mais: interpola entre os pontos vizinhos.
- Fora da faixa: limita ao extremo e sinaliza que a posição não foi calibrada.
- Falha breve de detecção: mantém a distância por até 200 ms desde a última pose válida.
- Sem distância após esse prazo, câmera desconectada ou imagem antiga: solicita `BASE_RPM`.
- O alvo automático positivo fica entre `AUTO_MIN_RPM` e `AUTO_MAX_RPM`;
  manual, calibração e tuners usam a faixa de zero a `MAX_RPM` (6000 RPM).

A distância usa a média central de até 40 amostras do último segundo, seguida
de suavização exponencial de 250 ms e banda morta de 1 cm. Isso reduz oscilações
e picos, com atraso na resposta ao movimento. A mira não mantém alvo perdido.

O controlador automático aplica banda morta de 10 RPM e intervalo mínimo de
100 ms, configurados em `Flywheel`. A malha dos motores continua sendo
executada a cada ciclo. A calibração manual aplica diretamente os passos,
inclusive 0,1 RPM. O controle limita a potência ao sentido de lançamento;
quando acima da RPM desejada, pode reduzir a potência até zero.

Registre novos pares ao alterar ângulo, câmera, geometria das tags, compressão, flywheel ou
elemento lançado. Esta calibração é de POLLEN; não foi medida para NECTAR.

Valide os pares nas duas alianças e com a CELL estabilizada. A tela mostra
os IDs e a aliança do alvo; inclua essa informação nas suas anotações.

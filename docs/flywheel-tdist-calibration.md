# Calibrar a flywheel com POLLEN e distância 3D até a abertura da CELL

## Primeiro teste

1. Abra o TeleOp **Flywheel Calibration - 3D**, grupo **Flywheel**. Ele não
   aciona o drivetrain; coloque o robô na posição de teste.
2. Confira a pipeline 8, aliança e CELL mostradas (azul 42–45 inicialmente).
   No gamepad 2, D-pad cima/baixo seleciona azul/vermelha e esquerda/direita
   seleciona CELL plateia/oposta. Aguarde `Distancia abertura (cm)` válida.
3. A RPM desejada começa em `Flywheel.BASE_RPM` (**2670** atualmente), com
   todos os mecanismos desligados. No gamepad 1, A alterna flywheel, B intake,
   X transfer e Y abre/fecha o trigger. Ajuste a RPM e o passo pelo D-pad.
4. Aguarde a RPM medida se aproximar da desejada e a distância estabilizar
   depois de mover o robô (a janela do filtro é de um segundo).
5. Com a flywheel na RPM desejada, ligue transfer com X e abra o trigger com Y
   para lançar; outro clique desliga/fecha cada um. B liga/desliga o intake.
   Os quatro mecanismos têm controles independentes.
6. Quando encontrar um bom ajuste, anote à mão a distância e a RPM desejada.
   Repita em outras posições. A tela permanece ao vivo, sem captura de pares.

| Controle (gamepad 1) | Ação |
| --- | --- |
| D-pad cima / baixo | Aumenta / diminui RPM desejada, um passo por clique |
| D-pad esquerda / direita | Diminui / aumenta o passo: 0,1; 1; 10; 100; 1000 RPM |
| A | Liga / desliga flywheel mantendo o ajuste |
| B | Liga / desliga intake |
| X | Liga / desliga transfer |
| Y | Abre / fecha trigger |

No gamepad 2, D-pad cima/baixo seleciona aliança azul/vermelha e
esquerda/direita seleciona CELL plateia/oposta, como no TeleOp principal.

A tela mostra somente distância, RPM desejada/medida, passo, estado dos
quatro mecanismos e alvo selecionado. A distância exibida já é filtrada e
usa a referência 3D da saída das bolas até a abertura da CELL.

O passo inicial é 10 RPM. Segurar D-pad/A/B/X/Y não repete o comando a cada loop.
A RPM fica entre zero e `Flywheel.MAX_RPM` (6000 inicialmente; configure conforme o launcher). `BASE_RPM` e `MAX_RPM` ficam em `Flywheel`; `PIPELINE_INDEX` e
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

**Os pares anteriores foram desativados com distância zero em `Flywheel`.**
Refaça os pares obtidos por `tdist`/POI ou da câmera até o centro do adesivo.
Até cadastrar os novos, o modo automático usa `BASE_RPM` e indica "Sem pares".
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

- Sem pares: usa `BASE_RPM` (2670 atualmente).
- Um par: mantém a RPM desse par; não existe curva medida ainda.
- Dois ou mais: interpola entre os pontos vizinhos.
- Fora da faixa: limita ao extremo e sinaliza que a posição não foi calibrada.
- Sem distância 3D válida: volta imediatamente ao pedido de `BASE_RPM`.
- `BASE_RPM` zero: desliga a RPM automática.

O controlador automático aplica banda morta de 10 RPM e intervalo mínimo de
100 ms, configurados em `Flywheel`. A malha dos motores continua sendo
executada a cada ciclo. A calibração manual aplica diretamente os passos,
inclusive 0,1 RPM. O controle limita a potência ao sentido de lançamento;
quando acima da RPM desejada, pode reduzir a potência até zero.

Registre novos pares ao alterar ângulo, câmera, geometria das tags, compressão, flywheel ou
elemento lançado. Esta calibração é de POLLEN; não foi medida para NECTAR.

Valide os pares nas duas alianças e com a CELL estabilizada. A tela mostra
os IDs e a aliança do alvo; inclua essa informação nas suas anotações.

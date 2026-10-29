# Calibrar a flywheel com POLLEN e distância 3D ao centro da CELL

## Primeiro teste

1. Abra o TeleOp **Flywheel Calibration - 3D**, grupo **Flywheel**. Ele não
   aciona o drivetrain; coloque o robô na posição de teste.
2. Confira a pipeline 8, aliança e CELL mostradas (azul 42–45 inicialmente).
   Clique no stick esquerdo para azul ou direito para vermelho; Y alterna
   CELL plateia/oposta. Aguarde `Distancia centro filtrada (cm)` válida.
3. A RPM desejada começa em **2720**, com a flywheel desligada. A liga/desliga;
   B desliga sem perder o ajuste. Ajuste a RPM e o passo pelo D-pad.
4. Aguarde a `Flywheel RPM medido` se aproximar da RPM desejada e a distância estabilizar
   depois de mover o robô (a janela do filtro é de um segundo).
5. Faça lançamentos com RT. LB alterna o intake; RB abre o dedinho enquanto
   pressionado. Verifique a repetibilidade com POLLEN.
6. Quando encontrar um bom ajuste, pressione X. `PAR ANOTADO` congela a
   distância **3D ao centro da CELL, filtrada em centímetros**, e a **RPM desejada**. Também
   mostra RPM medida, aliança, IDs da CELL e quantidade de amostras na captura.
7. Anote o par antes de sair. A captura fica apenas nesta execução; X não
   grava arquivo nem modifica o código. Repita em outras posições.

X registra a escolha manual; não verifica se o lançamento acertou ou se a
RPM já estava estabilizada. A telemetria ao vivo continua atualizando enquanto
o par capturado fica congelado. Se uma captura falhar, o último par permanece.

| Controle | Ação |
| --- | --- |
| D-pad cima / baixo | Aumenta / diminui RPM desejada, um passo por clique |
| D-pad esquerda / direita | Diminui / aumenta o passo: 0,1; 1; 10; 100; 1000 RPM |
| A | Liga / desliga flywheel mantendo o ajuste |
| B | Desliga flywheel e impede alimentação pelo RT |
| RT | Aciona transfer e trigger enquanto pressionado, com flywheel ligada |
| LB | Alterna intake |
| RB | Abre dedinho enquanto pressionado |
| Clique stick esquerdo / direito | Seleciona aliança azul / vermelha |
| Y | Alterna CELL da plateia / oposta dentro da aliança |
| X | Congela o par distância/RPM para anotação |

O passo inicial é 10 RPM. Segurar D-pad/A/Y/X não repete o comando a cada loop.
A RPM fica entre zero e `Flywheel.MAX_RPM` (6000 inicialmente; configure conforme o launcher). `BASE_RPM` e `MAX_RPM` ficam em `Flywheel`; `PIPELINE_INDEX` e
`UPSIDE_DOWN` ficam em `Limelight`. Ao parar o OpMode, a flywheel, o
transfer e o intake são desligados, os servos são fechados e a câmera é parada.

## Configuração da Limelight

Siga o [guia do painel e das AprilTags BIOBUZZ](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Limelight/README.md),
com pipeline 8, família 36h11, tamanho nominal 82,55 mm, filtros dos 16 IDs,
e Full3D. **Limelight OS 2026.0 é suficiente**; não é necessário atualizar
para 2026.1, configurar POIs ou fornecer `tdist` no JSON.

O Java calcula a distância **3D da câmera ao centro do cluster de tags da
CELL selecionada** a partir de `getTargetPoseCameraSpace()`. Inclui os três
eixos e os deslocamentos locais entre tags e centro; não é distância
horizontal nem medida da saída do launcher. Tags de outras CELLS são
ignoradas. A tabela usa a leitura filtrada convertida para centímetros.

Refaça os pares antigos obtidos por `tdist`/POI, pois a referência física
mudou. Calibração e modo automático agora usam o mesmo centro calculado em Java,
sem depender de qual tag a pipeline escolhe como principal. O cálculo é
exercitado com `LLResult` real do SDK FTC 12.0.0 e JSON sem `tdist`.

## Aplicar os pares

Edite `subsystems/FlywheelAdvanced/Flywheel.java`:

| Par | Distância ao centro filtrada (cm) | RPM desejada anotada |
| --- | --- | --- |
| 1 | `BASE_DISTANCE_CM` | `BASE_RPM` |
| 2 | `BASE_DISTANCE_2_CM` | `BASE_RPM_2` |
| 3 | `BASE_DISTANCE_3_CM` | `BASE_RPM_3` |

Distância zero desativa o par até a medição. Preencha distância e RPM de cada
par validado, sem duplicar distâncias. A tabela aceita qualquer ordem e copia
os valores na inicialização. Para mais pontos, adicione pares em
`Flywheel.createRpmTable()` e gere o programa novamente.

- Sem pares: usa `BASE_RPM` (2720 inicialmente).
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

Valide os pares nas duas alianças e com a CELL estabilizada. Os IDs do alvo
e a aliança ficam registrados na captura para conferir a referência usada.

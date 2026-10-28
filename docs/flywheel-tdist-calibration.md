# Calibrar a flywheel com POLLEN e tdist

## Primeiro teste

1. Abra o TeleOp **Flywheel Calibration - tdist**, grupo **Flywheel**. Ele não
   aciona o drivetrain; coloque o robô na posição de teste.
2. Confira a pipeline 8 e o hive mostrado (42–45 inicialmente; Y troca para
   38–41). Aguarde uma leitura válida de `tdist filtrado (cm)`.
3. A RPM desejada começa em **2720**, com a flywheel desligada. A liga/desliga;
   B desliga sem perder o ajuste. Ajuste a RPM e o passo pelo D-pad.
4. Aguarde a `Flywheel RPM medido` se aproximar da RPM desejada e a distância estabilizar
   depois de mover o robô (a janela do filtro é de um segundo).
5. Faça lançamentos com RT. LB alterna o intake; RB abre o dedinho enquanto
   pressionado. Verifique a repetibilidade com POLLEN.
6. Quando encontrar um bom ajuste, pressione X. `PAR ANOTADO` congela a
   distância **tdist filtrada em centímetros** e a **RPM desejada**. Também
   mostra RPM medida, hive e quantidade de amostras na captura.
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
| Y | Alterna hive 38–41 / 42–45 |
| X | Congela o par distância/RPM para anotação |

O passo inicial é 10 RPM. Segurar D-pad/A/Y/X não repete o comando a cada loop.
A RPM fica entre zero e `Flywheel.MAX_RPM` (6000 inicialmente; configure conforme o launcher). `BASE_RPM` e `MAX_RPM` ficam em `Flywheel`; `PIPELINE_INDEX` e
`UPSIDE_DOWN` ficam em `Limelight`. Ao parar o OpMode, a flywheel, o
transfer e o intake são desligados, os servos são fechados e a câmera é parada.

## Configuração da Limelight

- Use Limelight OS **2026.1 ou posterior**, que fornece `tdist` no JSON.
- Na **pipeline 8**, selecione o tipo AprilTag/Fiducial Markers e habilite o
  rastreamento 3D/Full3D. Confira a família e o **Marker Size em mm** para as
  tags realmente usadas no teste; não copie medidas de FRC.
- Defina um **mesmo ponto físico de referência** do HIVE. Se usar as quatro
  tags, configure os POIs de cada uma para esse mesmo ponto. No mapa, a opção
  `Use Map POI Offsets` usa os POIs por tag; confira também offsets globais de
  UI, que têm prioridade. Não presuma que o mapa padrão de outro jogo possui
  o centro correto do HIVE.
- Sem POI, `tdist` mede até a tag selecionada pela câmera. A troca de tag
  principal pode alterar essa referência, mesmo sem o robô se mover. Como
  alternativa inicial, teste sempre a mesma tag via **ID Filters**, mantendo
  a mesma configuração na calibração e no uso.
- O SDK não identifica no getter o alvo principal associado a `tdist`.
  Por isso, esta implementação aceita o valor somente quando **todas** as
  AprilTags do frame pertencem ao hive selecionado. Um frame com tags de
  ambos os hives é rejeitado para a RPM; a mira existente continua independente.
  Use ID Filters/câmera apontada para isolar o hive durante a calibração.

Não existe neste código um botão separado que “ative o tdist”. Com OS e 3D
compatíveis, ele é produzido pela câmera. O status da telemetria indica campo
ausente/inválido, falta de frame válido ou outro alvo no frame. A implementação
não substitui campo ausente por zero nem pela estimativa antiga.

`tdist` é distância **3D da câmera ao alvo/POI**, em metros no SDK; a telemetria
converte para centímetros. Não é a distância horizontal da saída da flywheel.
Esta tabela é experimental e usa diretamente `tdist`; ela não aplica a
equação balística à distância inclinada.

Referências: [tDist e POIs no OS 2026.1](https://docs.limelightvision.io/docs/docs-limelight/software-change-log),
[configuração de AprilTags](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags)
e [rastreamento 3D](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d).
O acesso por `LLResult.toString()` foi conferido no código-fonte do artefato
`org.firstinspires.ftc:Hardware:12.0.0` e em testes com a classe real.

## Aplicar os pares

Edite `subsystems/FlywheelAdvanced/Flywheel.java`:

| Par | tdist filtrado anotado (cm) | RPM desejada anotada |
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
- Sem tdist válido: volta imediatamente ao pedido de `BASE_RPM`.
- `BASE_RPM` zero: desliga a RPM automática.

O controlador automático aplica banda morta de 10 RPM e intervalo mínimo de
100 ms, configurados em `Flywheel`. A malha dos motores continua sendo
executada a cada ciclo. A calibração manual aplica diretamente os passos,
inclusive 0,1 RPM. O controle limita a potência ao sentido de lançamento;
quando acima da RPM desejada, pode reduzir a potência até zero.

Registre novos pares ao alterar ângulo, câmera/POI, compressão, flywheel ou
elemento lançado. Esta calibração é de POLLEN; não foi medida para NECTAR.

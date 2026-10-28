# Limelight 3A — painel e AprilTags BIOBUZZ

Configuração para `TeleOpMain` e `Flywheel Calibration - tdist`. Os valores
correspondentes do robô ficam no início de [Limelight.java](Limelight.java).

## 1. Abrir o painel

1. Conecte a Limelight ao computador por USB e abra
   [http://limelight.local:5801](http://limelight.local:5801). Se o nome não
   resolver, localize a câmera pelo Limelight Hardware Manager.
2. Use **Limelight OS 2026.1 ou posterior**, necessário para `tdist` no JSON.
3. Configure a **pipeline de índice 8**. O programa seleciona esse índice ao
   inicializar; trocar aliança/CELL no controle mantém a mesma pipeline.
4. No Control Hub, configure o dispositivo como **`limelight`**. O polling de
   100 Hz é solicitado pelo código e não representa a taxa de imagens da câmera.

Ao terminar, baixe a pipeline pelo botão de download do painel para guardar
uma cópia. Configure com o OpMode parado para evitar disputa pela pipeline.

## 2. Ajustes da pipeline 8

Os nomes podem variar conforme a versão do painel.

| Aba / ajuste | Valor para este projeto |
| --- | --- |
| Input / Pipeline Type | **Fiducial Markers / AprilTags** |
| Input / Source Image | **Camera** |
| Standard / Family | **AprilTag Classic 36h11** |
| Standard / Marker Size | **82.55 mm** (3,25 polegadas; confira a impressão real) |
| Standard / ID Filters | **`30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45`** |
| Rastreamento 3D | **Full 3D habilitado**, com pose de cada tag no espaço da câmera |
| Resolução | Comece pela maior disponível para 3D; confira latência e estabilidade |
| Detector Downscale | Comece em **1**; aumente se precisar de mais FPS e valide o alcance |
| Exposure / Gain | Ajuste na iluminação do campo para detectar em movimento, com pouca exposição |
| Cropping / Area | Comece sem recorte e sem excluir tags pequenas; ajuste após testar o alcance |
| Stream Orientation | Apenas conforto visual; não corrige os dados de mira |

`UPSIDE_DOWN = true` corresponde à montagem atual invertida. Se a montagem
mudar, ajuste a constante e confira o sentido da correção com o robô parado.
A mira usa as poses 3D individuais e o centro calculado em Java; calibrar o
crosshair `tx` no painel não substitui esse cálculo.

## 3. IDs e seleção no TeleOpMain

| Aliança | CELL | IDs | Seleção |
| --- | --- | --- | --- |
| Vermelha | Oposta à plateia | 30–33 | D-pad baixo + direita |
| Vermelha | Lado da plateia | 34–37 | D-pad baixo + esquerda |
| Azul | Lado da plateia | 38–41 | D-pad cima + esquerda |
| Azul | Oposta à plateia | 42–45 | D-pad cima + direita |

Pode selecionar cada eixo separadamente. **Cima seleciona azul; baixo,
vermelha; esquerda, plateia; direita, lado oposto.** A seleção permanece ao
soltar. Comandos opostos simultâneos mantêm aquele eixo. Inicial: **azul,
CELL oposta, 42–45**. Esquerda/direita são atalhos para as CELLS, não lados
relativos à orientação instantânea do robô.

A telemetria mostra `Alianca`, `CELL selecionada`, IDs, tags usadas e `tdist`.
Trocar aliança/CELL limpa as amostras do alvo anterior e atualiza a mira,
inclusive se a câmera ainda estiver entregando o mesmo timestamp.

**ID Filters no painel e seleção no Java têm papéis diferentes:** o painel
permite detectar os 16 IDs; o código usa apenas os quatro da CELL escolhida.
O D-pad não reescreve o campo ID Filters. Pode restringir o painel aos IDs de
uma aliança/CELL durante um teste, mas restaure a lista completa para permitir
todas as seleções acima.

## 4. Referência de distância para calibrar RPM

`tdist` mede em **metros** da câmera ao alvo principal ou POI ativo. A
telemetria converte para **centímetros**. Não é distância horizontal nem
medida a partir da saída da flywheel. Não existe um botão separado para ativar
`tdist`: confira OS, 3D e a saída de dados ao vivo do painel.

Escolha uma referência física e mantenha-a na calibração e no jogo:

- **Sem POI:** a referência é o centro da tag principal. Para um primeiro
  teste consistente, restrinja ID Filters a uma tag da CELL e registre qual
  foi usada. Com várias tags, a referência pode mudar quando a câmera troca
  de tag principal; a filtragem temporal não corrige essa diferença.
- **Com POIs por tag:** no Map Builder, configure cada tag do cluster para o
  mesmo ponto físico e habilite **Use Map POI Offsets**. Zere os offsets 3D
  globais de UI e eventuais overrides por NetworkTables, que têm prioridade.
  O formato `poi` do fmap é `[forward, left, down]`, em metros relativos à
  tag. Valide no visualizador; não copie diretamente os sinais do vetor X
  usado pelo cálculo Java. Verifique a referência cobrindo uma tag de cada
  vez, com o robô imóvel. Isso configura a distância; a mira Java permanece.

O SDK FTC 12 não fornece um getter que associe o `tdist` à ID principal. Por
isso, qualquer tag fora da CELL selecionada **invalida a distância desse
frame**, mesmo que uma tag correta também apareça. A mira ainda pode usar
as poses corretas. A telemetria pede para isolar as tags; a flywheel solicita
`BASE_RPM` enquanto não há distância válida. A lista completa no painel não
garante que todo enquadramento terá distância disponível: aponte para isolar
a CELL desejada e confirme o status antes de registrar pares.

## 5. Particularidades do BIOBUZZ

Conferido no manual oficial, seção 9.6/9.9, versão **TU06**, figuras 9-15–9-17,
em 28/10/2026:

- Os adesivos ficam sob as CELLS; a borda inferior aponta para o centro do
  campo. Use os arquivos oficiais de impressão, sem redimensionar desenhos
  do manual. Tamanho, ordem e alinhamento influenciam a pose 3D.
- Em cada adesivo, os IDs crescem da esquerda para a direita quando visto
  de frente. No desenho superior do campo o lado oposto aparece invertido.
  Os deslocamentos locais são iguais nas duas alianças; não se deve inverter
  os sinais só porque o alvo é vermelho.
- O cálculo preservado estima o **centro do cluster de tags**, não o centro
  da abertura de lançamento. Os centros das tags ficam a 6,5 e 2,75 polegadas
  de cada lado desse ponto; a pose transforma esses deslocamentos para a câmera.
- O HIVE gira e alterna a CELL voltada para cima. A seleção é manual e não
  detecta automaticamente qual CELL está pronta para receber lançamentos.
  Selecione a CELL adequada e espere estabilizar antes de calibrar/disparar.
  A janela de distância dura até um segundo; não representa posição instantânea
  durante a virada.
- Como as tags se movem com a estrutura, não use suas coordenadas como
  marcos fixos para `botpose`/MegaTag. Este código usa poses relativas à câmera
  e não exige localização global nem mapa de campo. Um fmap usado para POIs
  locais não torna essas tags fixas.

## 6. Conferência no robô

1. Aponte para uma CELL e confira os IDs no painel e a seleção na telemetria.
2. Confira uma pose 3D por tag e `tdist` positivo no Live Data/JSON. Se faltar
   pose, confira Full3D; se faltar `tdist`, confira OS 2026.1+ e os offsets.
3. Com flywheel desabilitada, teste as quatro seleções e cubra as tags aos
   poucos: o centro estimado deve continuar no mesmo ponto.
4. Ao selecionar outra aliança/CELL sem mover a câmera, confirme que as
   leituras do alvo antigo desaparecem. Depois enquadre o novo alvo.
5. Faça o [procedimento de calibração](../../../../../../../../../../docs/flywheel-tdist-calibration.md).
   Nesse TeleOp o D-pad ajusta RPM/passo: **clique no stick esquerdo para azul,
   direito para vermelho; Y alterna CELL**. A captura inclui aliança e IDs.
6. Valide os pares nas duas alianças e nas posições estáveis usadas. Não
   reutilize automaticamente os antigos 308 cm. Alterar câmera, referência/POI,
   launcher ou elemento exige conferir a calibração novamente.

## Fontes oficiais

- [FIRST — manual BIOBUZZ, seção 9 ARENA](https://ftc-resources.firstinspires.org/ftc/game/manual-09)
- [FIRST — recursos do campo e AprilTags imprimíveis](https://ftc-resources.firstinspires.org/ftc/field)
- [Limelight 3A — acesso, USB e hardware](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
- [Painel e pipelines](https://docs.limelightvision.io/docs/docs-limelight/getting-started/pipelines)
- [AprilTags — família, tamanho e ID Filters](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags)
- [Rastreamento 3D](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d)
- [OS 2026.1 — tdist e POIs por tag](https://docs.limelightvision.io/docs/docs-limelight/software-change-log)

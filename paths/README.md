# Arquivos do Pedro Visualizer

[AutoPrincipal.pp](AutoPrincipal.pp) e [AutoSecundario.pp.json](AutoSecundario.pp.json)
correspondem aos percursos de `AutoPrincipal.java` e `AutoSecundario.java`, no formato 2.0.0 do
[Pedro Visualizer](https://visualizer.pedropathing.com/).

[AutoTerciario.pp](AutoTerciario.pp) corresponde a `AutoTerciario.java`,
com cinco segmentos em cinco movimentos: preparacao da flower, coleta,
retorno curto, retorno ao lancamento e estacionamento. O grupo inicial
vazio do codigo exportado foi omitido. A sequencia inclui o disparo inicial
e o disparo depois da flower, alem das esperas de coleta e recolhimento.

Importe o arquivo `.pp` no Visualizer e selecione o mapa BIOBUZZ nas
configuracoes. Os 9 trechos do principal e secundario mantem os nomes, coordenadas, controles Bezier
e interpolacoes de heading usados no codigo. Os grupos sao:

- `ate_lancamento_2`: chegadinha inicial, preparacao/coleta do jardim e
  segundo lancamento.
- `preparacao_flower`: curva direta do segundo lancamento ate a preparacao
  da flower, com controle em `(67.82870967741937, 116.66903225806453)` e
  heading linear de 270 a 90 graus. Substitui a segunda chegadinha.
- `retorno_lancamento_3`: curva direta desde pos_flower, com tangente reversa
  e controle em `(67.4635, 117.3994)` no principal/secundario, ou
  `(68.0113, 117.3994)` no terciario. Substitui o pre_lancamento_3.

Coleta da flower, pos_flower e estacionamento permanecem trechos individuais.
As pausas fixas aparecem na sequencia: 4 s antes da saida inicial, 2,5 s em
cada lancamento seguinte, 2 s na coleta da flower e 0,5 s para recolher o
coletor. A espera restante para abrir os servos e as condicoes de parada
dependem da execucao do robo e nao sao reproduzidas pela simulacao.

As velocidades frontal e lateral foram copiadas da calibracao atual do
projeto. Dimensoes, velocidade angular, atrito e limites de aceleracao da
previsao foram mantidos do `.pp` antigo; o tempo previsto e aproximado.
O modificador `brakeAggression = 1.05`, a mira por Limelight e as acoes dos
mecanismos continuam no Java, pois nao fazem parte do formato do Visualizer.
As sobreposicoes dos gols do arquivo antigo, de DECODE, foram removidas.

Os lancamentos 2 e 3 continuam em `(60, 147)`, exatamente como no codigo,
mesmo ultrapassando o limite de 141,5 in do campo do Visualizer. Essas
coordenadas foram preservadas para voce poder revisa-las no editor.

Editar ou exportar este arquivo no Visualizer nao atualiza automaticamente
o OpMode. Ao trazer as alteracoes de volta, preserve a rotina de disparos,
flower e os grupos de deslocamento existentes no V3.

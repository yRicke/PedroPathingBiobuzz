# Organização dos subsistemas

Cada pasta reúne as classes do subsistema, seus OpModes de teste em `Tests/`
e seus ajustes em `Tunning/`, quando existentes. Os subsistemas recebem
`HardwareMap` e `Telemetry`, executam `run(...)` no loop e publicam seu estado
por `sendTelemetry()`. O TeleOp coordena entradas e chamadas dos subsistemas.

| Pasta | Responsabilidade |
| --- | --- |
| `Drive/` | Drive manual, field-centric, hold e heading pelo centro do HIVE |
| `Limelight/` | Validação da câmera, heading das AprilTags e distância tdist |
| `FlywheelAdvanced/` | Controle dos motores, tabela de RPM e estabilização do setpoint |
| `Intake/` | Coleta e ejeção |
| `Transfer/` | Alimentação dos elementos |
| `Trigger/` | Abertura do disparo |
| `DedinhoNervoso/` | Retenção/liberação pelo servo |

## TeleOp principal

`TeleOpMainTransferTest` usa o `Follower` criado por `pedro.Constants` com
Pinpoint e mecanum. A orientação ao apertar Play define a frente do campo.
O stick esquerdo controla a translação field-centric, o direito controla o giro,
e o gatilho esquerdo aplica modo precisão. Ao parar e soltar os sticks, o
Pedro mantém a pose. Right bumper ativa a mira com translação livre; right
trigger aciona transfer/trigger. Sem visão válida, o giro retorna ao piloto.

O TeleOp chama a Limelight, fornece sua distância ao controlador de RPM e
passa o setpoint resultante à flywheel. A mira recebe a observação de heading.

## Visão e distância

A configuração está em `Limelight/LimelightConfig.java`: dispositivo,
pipeline, montagem, polling, idade máxima e janela de distância. A seleção
é feita pelo D-pad: hive esquerdo 38–41, direito 42–45 (inicial).

`HiveCenterEstimator` projeta o centro físico do HIVE a partir das poses das
suas tags, considerando rotação e os deslocamentos horizontais conhecidos.
A média ponderada combina estimativas do centro, não os centros das tags
visíveis. A observação `AprilTagObservation` contém somente o estado da mira,
ângulo, quantidade de tags e timestamp.

`TdistReader` lê o campo `tdist` do JSON conservado por `LLResult.toString()`
no SDK FTC 12.0.0. Esta é a única fonte de distância. É a distância 3D da
câmera ao alvo/POI em metros. `getRawDistanceMeters()` expõe a leitura bruta;
`getDistanceMeters()` expõe a filtrada. As linhas da telemetria usam o nome
`tdist` e mostram centímetros. Frames com outros alvos além do HIVE selecionado
não fornecem distância, porque o SDK não identifica o alvo principal do tdist.

`DistanceFilter` recebe distância, timestamp e identidade do alvo diretamente.
Mantém até 40 frames distintos na janela de um segundo, separados por pelo
menos 25 ms. Remove 25% dos menores e dos maiores valores e calcula a média
restante (20 centrais com 40 amostras). Só recalcula quando a janela muda.
Perda de leitura ou troca de HIVE limpa o histórico. A câmera rejeita imagens
antigas, pipeline incorreta, desconexão e timestamps congelados por mais de 150 ms.
A decodificação e a estimativa do centro são feitas uma vez por frame/HIVE.

## Flywheel

`FlywheelConfig.java` reúne nomes dos motores, ticks por revolução,
transmissão, ganhos, limite de RPM, pares medidos e parâmetros do setpoint.
`FlywheelRpmTable` copia, valida e ordena a tabela na inicialização.
`FlywheelRpmController.run(...)` consulta essa tabela e estabiliza o setpoint
com banda morta de 10 RPM e intervalo mínimo de 100 ms. Ao perder a distância,
volta imediatamente à RPM base. Sem pares/um par usa RPM fixa; com dois ou
mais interpola e limita aos extremos, sinalizando posições fora da faixa.

`Flywheel.run(rpm)` mantém o controle do motor em todos os ciclos. O controle
é feedforward mais proporcional, com potência entre 0 e 1 no sentido de
lançamento. Overspeed reduz potência até zero. Os sinais físicos dos dois
motores são aplicados apenas nessa classe. O encoder do motor 1 fecha a malha;
o encoder 2 aparece na telemetria para diagnóstico. RPM/leitura principal
não finita desliga ambos os motores. `run(0)` e `stopMotors()` desligam.

Os tuners de kV/kP usam esse mesmo controlador e começam com os ganhos da
configuração. As medições aprovadas devem ser copiadas para `FlywheelConfig`.

O [procedimento de calibração](../../../../../../../../../docs/flywheel-tdist-calibration.md)
descreve o TeleOp de ajuste, a configuração da Limelight e os pares por tdist.

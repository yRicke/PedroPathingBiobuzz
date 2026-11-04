# Flower Collector com servos contínuos

O subsistema está em
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlowerCollector/FlowerCollector.java`.
Ele usa dois `CRServo` e controla o tempo de giro. O antigo tuner posicional
foi removido; não é necessário encontrar valores de posição de 0 a 1.

## Configuração

1. Com o robô desligado, conecte cada servo à sua porta SERVO no Control Hub ou
   Expansion Hub. Preserve os nomes de hardware **flowerCollect1** e
   **flowerCollect2** nas portas correspondentes.
2. Na Driver Station, abra **Configure Robot**, edite a configuração ativa,
   escolha o Hub e entre em **Servos**. Configure cada porta como
   **Continuous Rotation Servo/CRServo**, salve e confira a configuração ativa.
3. O modo interno do goBILDA também precisa ser contínuo. No Servo Programmer,
   conecte servo e alimentação, coloque a chave em **C** e dê um toque curto em
   **P**. O LED P pisca três vezes ao concluir. Trocar o tipo em Java ou na
   Driver Station não reprograma o modo interno. Consulte as
   [instruções oficiais do programador](https://www.gobilda.com/servo-programmer-for-2000-series-dual-mode-servo/)
   e a [ficha do servo de cinco voltas](https://www.gobilda.com/2000-series-5-turn-dual-mode-servo-25-2-torque/).
4. Os dois servos começam com `DcMotorSimple.Direction.FORWARD`. Para inverter
   um lado, altere apenas a linha `setDirection(...)` daquele servo no construtor
   para `DcMotorSimple.Direction.REVERSE`.

## Ajustar somente o tempo

No topo da classe:

```java
public static final double SERVO_RUN_TIME_SECONDS = 1.0;
```

Esse valor é em segundos, precisa ser positivo e finito e vale para abrir e
fechar os dois servos. `open()` gira ambos com potência +1; `close()` gira
ambos com potência -1. Ao completar o tempo, o subsistema envia potência zero.
Começa parado no construtor, sem abrir nem fechar automaticamente.

O tempo é acompanhado pelo loop, sem `sleep` e sem thread em segundo plano.
O corte acontece no primeiro ciclo de `update()` após o prazo. Se o loop parar
de executar, o subsistema não consegue cortar a potência por conta própria.

## Uso com comandos por evento

Crie `FlowerCollector(hardwareMap, telemetry)` na inicialização. No loop,
identifique um toque no botão desejado para chamar `open()` ou `close()` e
mantenha `update()` em todos os ciclos, mesmo sem botão pressionado:

```java
if (solicitouAbrir) flowerCollector.open();
else if (solicitouFechar) flowerCollector.close();

flowerCollector.update();
flowerCollector.sendTelemetry();
```

`solicitouAbrir` e `solicitouFechar` representam os eventos definidos pelo seu
OpMode; o subsistema ainda não está integrado aos botões do `TeleOpMain`.
No `finally`/`stop()` do OpMode, chame `flowerCollector.stop()` para zerar os dois
servos também em um encerramento antecipado.

Se o OpMode já mantém um booleano de alvo, pode chamar em todos os ciclos:

```java
flowerCollector.run(fechado); // true fecha, false abre; ja executa update()
```

Chamadas repetidas para o mesmo alvo não reiniciam o cronômetro nem repetem
o giro depois de concluído. Mudar o alvo durante o giro inverte os dois servos
imediatamente e inicia o tempo completo no novo sentido. Para repetir o mesmo
movimento sem passar pelo comando oposto, chame `stop()` antes da nova solicitação.

## O que o controle por tempo garante

O subsistema limita a duração do comando; não mede a posição nem detecta
batentes. Ao zerar a potência, cessa o giro comandado, mas não existe uma malha
para sustentar a posição contra carga. A posição final pode variar com carga,
tensão e movimento interrompido. A telemetria informa comando, movimento e
tempo restante, sem afirmar que o mecanismo atingiu uma posição física.

O batente do mecanismo limita o curso, mas não desliga o servo: se ele chegar
antes do prazo, continuará recebendo potência até o corte. Ajuste o tempo para
evitar ficar forçando. Confirme os sentidos inicialmente com o mecanismo
desacoplado; depois verifique o tempo no mecanismo montado.

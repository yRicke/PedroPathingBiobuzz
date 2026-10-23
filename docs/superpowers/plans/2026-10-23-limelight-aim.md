# Limelight Aim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Centralizar a AprilTag selecionada enquanto o gatilho mantém mira e disparo ativos.

**Architecture:** A Limelight entrega uma observação imutável, filtrada por ID,
pipeline e idade. PedroDrive mantém o ponto X/Y e atualiza somente o heading
usando o Follower existente. O TeleOp seleciona exclusivamente drive manual ou mira.

**Tech Stack:** Java 8, FTC SDK 12.0.0, Pedro 3.0.1, JUnit 4.13.2, Mockito 4.11.0 nos testes de hardware.

**Spec:** `docs/superpowers/specs/2026-10-23-limelight-aim-design.md`

## Global Constraints

- D-pad esquerdo: 39; direito: 43; inicial: 43; ambos preservam a seleção.
- Montagem normal/invertida e pipeline 0 configuráveis no TeleOp; hardware `limelight`.
- `right_trigger` do gamepad 1 mira e dispara juntos; preservar demais mecanismos.
- Idade máxima: 150 ms; tolerância: 1 grau; correção máxima: 8 graus/frame; busca: 2 graus.
- Preservar a edição preexistente em `TeleOpMainBrabo.java`.

## Review Focus

- ID diferente do selecionado: procurar, sem mirar outra tag.
- Leitura repetida: não acumular a correção de heading.
- Desconexão durante busca: parar a busca e manter o ponto capturado.
- Soltura com sticks neutros: cancelar a rotação anterior imediatamente.
- Montagem invertida: corrigir a direção do giro sem afetar o drive manual.

## Task 1: Observações da Limelight

**Files:** criar `subsystems/Limelight/AprilTagObservation.java`,
`subsystems/Limelight/Limelight.java` e o teste correspondente em `src/test/java`;
adicionar Mockito como `testImplementation` em `TeamCode/build.gradle`.
Todos os caminhos de subsistemas são relativos ao package `org.firstinspires.ftc.teamcode`.

**Interfaces:** `Limelight.run(boolean, boolean)`, `getObservation()`,
`sendTelemetry()`, `stop()`; observações `unavailable(id)`, `searching(id, timestamp)`
e `detected(id, txDegrees, timestamp)` com getters imutáveis.

- [x] Escrever testes: seleção 39/43 persistente, ambos, erro da tag correta,
  inversão de sinal, timestamp não finito, pipeline errada, idade 150/151 ms,
  desconexão, câmera ausente e ausência da tag.
- [x] Rodar `gradlew.bat :TeamCode:testDebugUnitTest --tests '*LimelightTest'`;
  esperar falha pela ausência da implementação.
- [x] Implementar leitura, validação e lifecycle da câmera, sem rede no loop
  além da leitura do resultado já disponível no SDK.
- [x] Rodar os testes do subsistema; esperar zero falhas.
- [ ] Commit em português: `feat (limelight): adiciona seleção e leitura de AprilTags.`

## Task 2: Mira com hold e integração

**Files:** modificar `subsystems/Drive/PedroDrive.java`, seu teste,
`TeleOp/TeleOpMainTransferTest.java` e `subsystems/README.md`.

**Interfaces:** consome `AprilTagObservation`; produz `PedroDrive.aim(observation)`.
`run(axial, lateral, yaw, precisionMode)` cancela a mira antes de retomar o manual.

- [ ] Escrever testes com Follower real: X/Y capturados, heading com erro de
  10 graus limitado a 8, busca de 2 graus, erro de 0.5 grau alinhado, frame
  repetido, novo frame, troca de ID, desconexão, soltura e nova captura.
- [ ] Rodar teste PedroDrive e observar a falha antes da implementação.
- [ ] Implementar os estados manual/busca/rastreio/alinhado/sem visão.
- [ ] Integrar a seleção por D-pad e o gatilho, preservando disparo e cleanup.
  Documentar montagem, pipeline, telemetria e procedimento de teste físico.
- [ ] Rodar `gradlew.bat test :TeamCode:assembleDebug`; esperar sucesso e
  zero falhas nas variantes debug/release.
- [ ] Fazer uma revisão independente do conjunto e tratar achados relevantes.
- [ ] Commit em português: `feat (teleop): integra mira de AprilTag com hold do Pedro.`

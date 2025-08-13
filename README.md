# Phys3D

**Phys3D** é um projeto didático de **física 3D em Java** com visualização em **JavaFX**.
Ele inclui corpos rígidos básicos (esfera/caixa/plano), detecção e resposta a colisões, atrito (estático/dinâmico), integração com **time step fixo + substeps**, *broadphase* por **grade uniforme**, e um sistema de **sleeping**.

A visualização usa um *viewer* 3D com câmera orbital, grid no chão, eixos XYZ, HUD de FPS e interação com o mouse 
(inclui “arremessar” esferas com clique direito).

---

## ✨ Funcionalidades

* Corpos rígidos: **Esfera**, **Caixa (AABB)** e **Plano infinito**.
* Colisões: esfera–plano, esfera–esfera, caixa–caixa (mínimo necessário).
* **Resposta por impulso** com restituição e **correção de penetração**.
* **Atrito estático e dinâmico** com clamp de impulso tangencial.
* **Time step fixo** com **substeps** e número de **iterações** do solver configuráveis.
* **Broadphase** com **Grade Uniforme** para reduzir *pairs* testados.
* **Sleeping**: corpos entram em repouso estável e “acordam” em contato/impulso.
* Viewer JavaFX: **câmera orbital**, **grid xadrez**, **eixos XYZ**, **HUD de FPS**.
* **Clique direito**: arremessa uma **esfera** na direção do ponto clicado.

---

## 🧰 Requisitos

* **Java 21+**
* **Maven 3.9+**
* **JavaFX 21.0.2** (resolvido automaticamente pelo plugin via Maven)

---

## ▶️ Como rodar

### Terminal

```bash
mvn -q javafx:run
```

---

## 🎮 Controles no Viewer

* **Mouse (arrastar)**: orbitar câmera
* **Shift + arrastar**: *pan* (transladar a câmera paralela à tela)
* **Scroll**: zoom
* **R**: resetar a cena
* **Espaço**: dar um “empurrão” na primeira esfera
* **G**: alternar visibilidade do grid
* **Clique direito**: **arremessa uma esfera** na direção do ponto clicado
* **HUD**: mostra **FPS** e status do **Grid**

---

## 🧪 Testes

Rode os testes com:

```bash
mvn test
```

Coberturas principais:

* **Colisão simples**: esfera–chão (queda e *bounce*)
* **Time step fixo + substeps**: penetração reduzida vs. *baseline*
* **Empilhamento simples**: duas caixas
* **Atrito**: desaceleração e deslizamento esperado
* **Broadphase (Grid)**: pares testados reduzidos vs. força bruta
* **Sleeping**: dormir em repouso e acordar com impulso

---

## ⚙️ Parâmetros úteis (afinando a simulação)

No **World**:

* `setFixedTimeStep(1.0/120.0)` — *time step* fixo (sugestão 120 Hz)
* `setSubsteps(4)` — subpassos por *frame* visual
* `setSolverIterations(6..10)` — iterações por contato
* `setSleepVelThreshold(…)` e `setSleepTime(…)` — critérios de *sleeping*

No **Collision** (ajustes globais):

* `setPositionCorrection(percent, slop)` — correção de penetração
* `setNormalImpulseVSlop(vSlop)` — zona morta para evitar jitter
* `setWakeThresholds(vWake, pWake)` — limiares para acordar em contato

Por corpo:

* `setRestitution(r)` — quicada (0..1)
* `setFrictionStatic(fs) / setFrictionDynamic(fd)` — atritos
* `setLinearDamping(d)` — amortecimento linear
* `setVelocity(vec)` / `applyImpulse(vec)` — inicialização/interação

Viewer (spawn de clique):

* `THROW_SPEED`, `THROW_RADIUS`, `THROW_MASS` — constantes no `PhysViewerApp`

---

## 🧩 Dicas de uso

* Se a esfera “nasce” muito perto/atravessa, aumente o *offset* de spawn:
  no `PhysViewerApp`, em `throwSphereFromClick`, altere `dirPhys.mul(0.30)` (30 cm).
* Para uma sensação mais “pesada”, aumente `THROW_MASS` e reduza `THROW_SPEED`.
* Se houver leve *jitter* em repouso, aumente `Collision.setNormalImpulseVSlop(...)` e/ou `setPositionCorrection(..., slop)`.

---

## ❓ Troubleshooting

* **Câmera confusa / grid “estranho”**
  Use **Scroll** para ajustar distância, **Shift+arrastar** para *pan*, **R** para reset.
  O grid está em **y=0**; eixos: X=vermelho, Y=verde, Z=azul.

---

## 🛣️ Roadmap (ideias)

* Gizmos de **normais/impulsos** em contato (linhas/setas).
* **Fricção rotacional** e spin.
* **Convexos genéricos** (GJK/EPA) e *sweep tests*.
* **Joints** simples (pivô, distância).

---
package phys.core;

import java.util.ArrayList;
import java.util.List;
import phys.broadphase.Broadphase;
import phys.collision.*;
import phys.math.Vec3;

/**
 * Representa o mundo físico 3D, responsável por gerenciar corpos rígidos, gravidade, colisões,
 * integração temporal e sistema de sleep. Permite configuração de timestep fixo, substeps,
 * broadphase e parâmetros globais de repouso.
 */
public final class World {
    private final List<RigidBody> bodies = new ArrayList<>();
    private Vec3 gravity = new Vec3(0, -9.81, 0);

    // Novo controle de timestep fixo
    private double fixedTimeStep = 1.0 / 120.0; // 120 Hz
    private double accumulator = 0.0;
    private int substeps = 1;
    private int solverIterations = 4;
    private Broadphase broadphase = null;
    // --- Parâmetros globais de sleep ---
    private double sleepVelThreshold = 0.05; // m/s
    private double sleepTime = 0.5;          // s sob o limiar para dormir
    // ==== DEBUG flags/estado (adicione no topo da classe World) ====
    // Ligue/desligue logs compactos por step:
    public static boolean DEBUG_SLEEP_LOGS = false;
    private double debugLogAccum = 0.0;

    /**
     * Define o limiar de velocidade linear abaixo do qual o corpo é considerado "parado".
     *
     * @param v Velocidade mínima (m/s) para considerar o corpo em movimento.
     */
    public void setSleepVelThreshold(double v) {
        this.sleepVelThreshold = Math.max(0.0, v);
    }

    /**
     * Define o tempo contínuo necessário (s) sob o limiar para entrar em sleep.
     *
     * @param seconds Tempo em segundos.
     */
    public void setSleepTime(double seconds) {
        this.sleepTime = Math.max(0.0, seconds);
    }

    /**
     * Define o vetor de gravidade global do mundo.
     *
     * @param g Vetor de gravidade.
     */
    public void setGravity(Vec3 g) {
        this.gravity = g;
        Collision.setGravity(g);
    }

    /**
     * Retorna o vetor de gravidade atual do mundo.
     *
     * @return Vetor de gravidade.
     */
    public Vec3 gravity() {
        return gravity;
    }

    /**
     * Retorna a lista de corpos rígidos presentes no mundo.
     *
     * @return Lista de RigidBody.
     */
    public List<RigidBody> bodies() {
        return bodies;
    }

    /**
     * Adiciona um corpo rígido ao mundo.
     *
     * @param b Corpo a ser adicionado.
     */
    public void addBody(RigidBody b) {
        bodies.add(b);
    }


    /**
     * Define o número de iterações do solver de contatos por substep. Mais iterações ajudam em empilhamentos estáveis
     * (custo extra de CPU).
     *
     * @param iterations >= 1
     */
    public void setSolverIterations(int iterations) {
        this.solverIterations = Math.max(1, iterations);
    }

    /**
     * Define o tamanho do passo fixo da simulação.
     *
     * @param step Tamanho do passo em segundos (ex: 1/120.0 para 120Hz).
     */
    public void setFixedTimeStep(double step) {
        this.fixedTimeStep = Math.max(1e-6, step);
    }

    /**
     * Define quantos substeps internos serão realizados para cada passo fixo.
     * <br>
     * Maior valor costuma reduzir penetração e jitter, mas custa performance.
     *
     * @param substeps Número de substeps (>=1).
     */
    public void setSubsteps(int substeps) {
        this.substeps = Math.max(1, substeps);
    }


    /**
     * Define a estratégia de broadphase (ex.: new UniformGridBroadphase(cellSize)). Passe null para voltar ao
     * brute-force O(n²).
     *
     * @param bp Instância de Broadphase ou null.
     */
    public void setBroadphase(phys.broadphase.Broadphase bp) {
        this.broadphase = bp;
    }

    /**
     * Atualiza o mundo usando timestep fixo e substeps internos.
     * <br>
     * Faz clamp de deltaTime para evitar a "espiral da morte" quando há um spike de frame.
     * <br>
     * Limita o número de passos fixos por chamada.
     *
     * @param deltaTime Tempo real decorrido desde a última chamada (segundos).
     */
    public void update(double deltaTime) {
        // clamp para evitar acumular tempo absurdo num frame travado
        double maxDeltaTime = 0.25; // 250 ms; ajuste se quiser
        double dt = Math.min(deltaTime, maxDeltaTime);

        accumulator += dt;

        int maxFixedStepsPerUpdate = 8; // evita laços longos demais por update
        int steps = 0;

        while (accumulator >= fixedTimeStep && steps < maxFixedStepsPerUpdate) {
            double subDt = fixedTimeStep / substeps;
            for (int i = 0; i < substeps; i++) {
                step(subDt);
            }
            accumulator -= fixedTimeStep;
            steps++;
        }

        // se estourou o máximo de passos, joga fora o excesso para manter a sim em dia
        if (steps == maxFixedStepsPerUpdate) {
            accumulator = 0.0;
        }
    }

    /**
     * Executa um único passo de simulação com tamanho de passo fixo.
     * Ordem importante: integra -> soft-contact -> colisões -> sleep.
     *
     * @param dt Tamanho do passo fixo (segundos).
     */
    public void step(double dt) {
        Collision.setCurrentDt(dt);

        // 0) Reset de atividade por corpo (para o sistema de sleep)
        for (RigidBody b : bodies) {
            b.beginStepActivityReset();
        }
        Collision.debugBeginStep();

        // 1) Integra (corpos acordados)
        for (RigidBody b : bodies) {
            b.integrate(gravity, dt);
        }

        // 2) Broadphase → pares candidatos
        java.util.List<phys.broadphase.Broadphase.RigidBodyPair> candidates;
        if (broadphase != null) {
            broadphase.clear();
            double[] min = new double[3], max = new double[3];
            for (RigidBody b : bodies) {
                if (Bounds.compute(b, min, max)) {
                    broadphase.insert(b, min, max);
                }
            }
            candidates = broadphase.computePairs();

            // adiciona pares com planos (dinâmicos × planos)
            java.util.List<RigidBody> planes = new java.util.ArrayList<>();
            java.util.List<RigidBody> others = new java.util.ArrayList<>();
            for (RigidBody b : bodies) {
                if (b.shape() instanceof Plane) planes.add(b);
                else others.add(b);
            }
            for (RigidBody dyn : others) {
                for (RigidBody pl : planes) {
                    candidates.add(new phys.broadphase.Broadphase.RigidBodyPair(dyn, pl));
                }
            }
        } else {
            candidates = new java.util.ArrayList<>();
            int n = bodies.size();
            for (int i = 0; i < n; i++) {
                for (int j = i + 1; j < n; j++) {
                    candidates.add(new phys.broadphase.Broadphase.RigidBodyPair(bodies.get(i), bodies.get(j)));
                }
            }
        }

        // 3) PRIMEIRO: aplicar soft-contact para Sphere–Plane (limpa vₙ e faz snap)
        for (var pair : candidates) {
            var a = pair.a();
            var b = pair.b();
            if (a.isSleeping() && b.isSleeping()) continue;

            if (a.shape() instanceof Sphere && b.shape() instanceof Plane) {
                Collision.softContactSpherePlane(a, b);
            } else if (b.shape() instanceof Sphere && a.shape() instanceof Plane) {
                Collision.softContactSpherePlane(b, a);
            }
        }

        // 4) Agora sim: resolver colisões com múltiplas iterações
        for (int iter = 0; iter < solverIterations; iter++) {
            for (var pair : candidates) {
                var a = pair.a();
                var b = pair.b();

                if (a.isSleeping() && b.isSleeping()) continue;

                // Sphere–Sphere
                var m1 = Collision.test(a, b);
                if (m1 != null) { Collision.resolve(m1); continue; }

                // Sphere–Plane
                var m2 = Collision.testSpherePlane(a, b);
                if (m2 != null) { Collision.resolve(m2); continue; }
                var m3 = Collision.testSpherePlane(b, a);
                if (m3 != null) { Collision.resolve(m3); continue; }

                // Sphere–AABB
                var m4 = Collision.testSphereAABB(a, b);
                if (m4 != null) { Collision.resolve(m4); continue; }
                var m5 = Collision.testSphereAABB(b, a);
                if (m5 != null) { Collision.resolve(m5); continue; }

                // AABB–Plane
                var m6 = Collision.testAABBPlane(a, b);
                if (m6 != null) { Collision.resolve(m6); continue; }
                var m7 = Collision.testAABBPlane(b, a);
                if (m7 != null) { Collision.resolve(m7); continue; }

                // AABB–AABB
                var m8 = Collision.testAABBAABB(a, b);
                if (m8 != null) { Collision.resolve(m8); }
            }
        }

        // 5) Sleep: thresholds de “silêncio” (pode manter estes)
        double impulseQuiet = 1e-2;     // Ns
        double correctionQuiet = 2e-3;  // m
        for (RigidBody b : bodies) {
            b.accumulateSleepTimer(dt, sleepVelThreshold, sleepTime, impulseQuiet, correctionQuiet);
        }

        // 6) LOG compacto a cada ~0.1s (se ligado)
        if (DEBUG_SLEEP_LOGS) {
            debugLogAccum += dt;
            if (debugLogAccum >= 0.1) {
                System.out.printf("[SLEEPDBG] resolves=%d softContacts=%d bodies=%d%n",
                    Collision.DEBUG_resolvesThisStep,
                    Collision.DEBUG_softContactsThisStep,
                    bodies.size());
                for (RigidBody b : bodies) {
                    String shape = b.shape().getClass().getSimpleName();
                    double v = b.velocity().length();
                    double vAvg = b.debugVelAvg();
                    double timer = b.debugSleepTimer();
                    boolean hc = b.hadContactThisStep();
                    double j = b.maxImpulseThisStep();
                    double corr = b.maxCorrectionThisStep();
                    boolean sl = b.isSleeping();
                    int id = System.identityHashCode(b);
                    System.out.printf("  id=%08x %-6s y=%7.4f v=%7.4f vAvg=%7.4f timer=%5.3f contact=%s jMax=%7.5f corrMax=%7.5f sleeping=%s%n",
                        id, shape, b.position().y(), v, vAvg, timer, hc, j, corr, sl);
                }
                debugLogAccum = 0.0;
            }
        }
    }


    /**
     * Zera o acumulador de tempo do timestep fixo.
     * <br>
     * Útil ao pausar/despausar a simulação ou quando alterar drasticamente o relógio.
     */
    public void resetAccumulator() {
        this.accumulator = 0.0;
    }

    /**
     * Retorna o fator de interpolação (0..1) representando a fração de tempo acumulada desde o último passo fixo. Útil
     * para interpolar posições na camada de renderização.
     * <br>
     * Ex.: renderPos = lerp(prevPos, currPos, getInterpolationAlpha()).
     *
     * @return Fator de interpolação (0 a 1).
     */
    public double getInterpolationAlpha() {
        if (fixedTimeStep <= 0.0) {
            return 0.0;
        }
        double alpha = accumulator / fixedTimeStep;

        if (alpha < 0.0) {
            return 0.0;
        }

        if (alpha > 1.0) {
            return 1.0;
        }

        return alpha;
    }

    /**
     * Cria um corpo dinâmico do tipo esfera com centro em {@code pos} e raio {@code radius}.
     *
     * @param pos Centro da esfera
     * @param radius Raio da esfera (>0)
     * @param mass Massa do corpo (>0)
     * @return Novo RigidBody dinâmico do tipo esfera.
     */
    public static RigidBody dynamicSphere(Vec3 pos, double radius, double mass) {
        return new RigidBody(pos, mass, new Sphere(radius));
    }

    /**
     * Cria um corpo estático do tipo plano definido por uma normal {@code normal} e distância {@code d} do plano à
     * origem (0,0,0).
     * <br>
     * O plano é definido pela equação n·x = d, onde n é a normal e x é um ponto no plano.
     *
     * @param normal Normal do plano (unitária)
     * @param d Distância do plano à origem
     * @return Novo RigidBody estático do tipo plano.
     */
    public static RigidBody staticPlane(Vec3 normal, double d) {
        return new RigidBody(new Vec3(0, 0, 0), 0.0, new Plane(normal, d));
    }

    /**
     * Cria um corpo dinâmico do tipo AABB com centro em {@code pos}.
     *
     * @param pos centro
     * @param halfExtents semi-eixos (hx, hy, hz)
     * @param mass massa (>0)
     * @return Novo RigidBody dinâmico do tipo caixa (AABB).
     */
    public static RigidBody dynamicBox(Vec3 pos, Vec3 halfExtents, double mass) {
        return new RigidBody(pos, mass, new AABB(halfExtents));
    }

}

package phys.core;

import phys.collision.Shape;
import phys.math.Vec3;

/**
 * Representa um corpo rígido 3D para simulação física, com suporte a forças, atrito, repouso (sleep) e integração.
 * Permite configuração de massa, forma, coeficientes de restituição e atrito, além de gerenciamento de estado de sono.
 */
public final class RigidBody {
    private Vec3 position;
    private Vec3 velocity = Vec3.ZERO;
    private Vec3 forceAccum = Vec3.ZERO;
    private final double mass;
    private final double invMass;
    private final Shape shape;
    private double restitution = 0.4; // “quique” básico
    private double frictionStatic = 0.6;
    private double frictionDynamic = 0.4;
    private boolean canSleep = true;
    private boolean sleeping = false;
    private double sleepTimer = 0.0;

    // Damping linear (s⁻¹). Ex.: 0.1 significa ~10%/s (dec decaimento exponencial).
    private double linearDamping = 0.05;

    // Filtro e histerese
    private double velAvg = 0.0; // EMA de |v|
    private static final double EMA_ALPHA = 0.2; // suavização (0..1); maior = responde mais rápido

    // Acúmulo de atividade no passo atual
    private boolean hadContactThisStep = false;
    private double maxImpulseThisStep = 0.0;
    private double maxCorrectionThisStep = 0.0;

    /**
     * Cria um novo corpo rígido.
     *
     * @param position Posição inicial do corpo.
     * @param mass Massa do corpo (kg). Se for 0, o corpo é considerado estático.
     * @param shape Forma geométrica usada para colisão.
     */
    public RigidBody(Vec3 position, double mass, Shape shape) {
        this.position = position;
        this.mass = mass;
        this.invMass = (mass > 0) ? 1.0 / mass : 0.0; // 0 => estático
        this.shape = shape;
    }

    /**
     * Indica se o corpo é estático (massa infinita).
     *
     * @return true se o corpo for estático, false caso contrário.
     */
    public boolean isStatic() {
        return invMass == 0.0;
    }

    /**
     * Adiciona uma força ao corpo (acumulada até o próximo passo de simulação).
     *
     * @param f Força a ser aplicada.
     */
    public void addForce(Vec3 f) {
        forceAccum = forceAccum.add(f);
    }

    /**
     * Limpa todas as forças acumuladas.
     */
    public void clearForces() {
        forceAccum = Vec3.ZERO;
    }

    /**
     * Retorna a posição atual do corpo.
     *
     * @return Posição (Vec3).
     */
    public Vec3 position() {
        return position;
    }

    /**
     * Define a posição do corpo.
     *
     * @param p Nova posição.
     */
    public void setPosition(Vec3 p) {
        position = p;
    }

    /**
     * Retorna a velocidade atual do corpo.
     *
     * @return Velocidade (Vec3).
     */
    public Vec3 velocity() {
        return velocity;
    }

    /**
     * Define a velocidade do corpo.
     *
     * @param v Nova velocidade.
     */
    public void setVelocity(Vec3 v) {
        velocity = v;
    }

    /**
     * Retorna o inverso da massa do corpo (0 para corpos estáticos).
     *
     * @return Inverso da massa.
     */
    public double invMass() {
        return invMass;
    }

    /**
     * Retorna a forma geométrica do corpo.
     *
     * @return Shape associado.
     */
    public Shape shape() {
        return shape;
    }

    /**
     * Retorna o coeficiente de restituição (elasticidade).
     *
     * @return Coeficiente de restituição (0 a 1).
     */
    public double restitution() {
        return restitution;
    }

    /**
     * Define o coeficiente de restituição (elasticidade).
     *
     * @param r Valor entre 0 e 1.
     */
    public void setRestitution(double r) {
        restitution = Math.max(0, Math.min(1, r));
    }

    /**
     * Retorna o coeficiente de atrito estático.
     *
     * @return Coeficiente de atrito estático.
     */
    public double frictionStatic() {
        return frictionStatic;
    }

    /**
     * Define o coeficiente de atrito estático (>=0).
     *
     * @param muS Novo coeficiente de atrito estático.
     */
    public void setFrictionStatic(double muS) {
        this.frictionStatic = Math.max(0.0, muS);
    }

    /**
     * Retorna o coeficiente de atrito dinâmico (cinético).
     *
     * @return Coeficiente de atrito dinâmico.
     */
    public double frictionDynamic() {
        return frictionDynamic;
    }

    /**
     * Define o coeficiente de atrito dinâmico (>=0).
     *
     * @param muK Novo coeficiente de atrito dinâmico.
     */
    public void setFrictionDynamic(double muK) {
        this.frictionDynamic = Math.max(0.0, muK);
    }

    /**
     * Retorna o damping linear (s⁻¹).
     *
     * @return Valor do damping linear.
     */
    public double linearDamping() {
        return linearDamping;
    }

    /**
     * Define o damping linear (s⁻¹). Valores pequenos (0.0–0.2) são razoáveis.
     *
     * @param damping Novo valor de damping linear.
     */
    public void setLinearDamping(double damping) {
        this.linearDamping = Math.max(0.0, damping);
    }

    /**
     * Indica se o corpo pode dormir (sleep).
     *
     * @return true se pode dormir, false caso contrário.
     */
    public boolean canSleep() {
        return canSleep;
    }

    /**
     * Habilita/desabilita a capacidade de dormir. Se desabilitar, acorda o corpo.
     *
     * @param canSleep true para permitir sleep, false para desabilitar.
     */
    public void setCanSleep(boolean canSleep) {
        this.canSleep = canSleep;
        if (!canSleep) {
            this.sleeping = false;
        }
    }

    /**
     * Retorna se o corpo está dormindo (não é integrado nem colidido).
     *
     * @return true se está dormindo, false caso contrário.
     */
    public boolean isSleeping() {
        return sleeping;
    }

    /**
     * Acorda imediatamente o corpo (volta a ser simulado normalmente).
     */
    public void wakeUp() {
        this.sleeping = false;
        this.sleepTimer = 0.0;
    }

    /**
     * Zera marcadores de atividade no início de cada step fixo.
     */
    public void beginStepActivityReset() {
        hadContactThisStep = false;
        maxImpulseThisStep = 0.0;
        maxCorrectionThisStep = 0.0;
    }

    /**
     * Marca que houve contato neste step.
     */
    public void markContact() {
        hadContactThisStep = true;
    }

    /**
     * Acumula a maior magnitude de impulso visto neste step.
     *
     * @param jMag Magnitude do impulso.
     */
    public void accumulateImpulse(double jMag) {
        if (jMag > maxImpulseThisStep) {
            maxImpulseThisStep = jMag;
        }
    }

    /**
     * Acumula a maior correção posicional vista neste step.
     *
     * @param corrMag Magnitude da correção posicional.
     */
    public void accumulateCorrection(double corrMag) {
        if (corrMag > maxCorrectionThisStep) {
            maxCorrectionThisStep = corrMag;
        }
    }

    /**
     * Indica se houve contato neste step.
     *
     * @return true se houve contato, false caso contrário.
     */
    public boolean hadContactThisStep() {
        return hadContactThisStep;
    }

    /**
     * Maior impulso visto neste step (Ns).
     *
     * @return Valor do maior impulso.
     */
    public double maxImpulseThisStep() {
        return maxImpulseThisStep;
    }

    /**
     * Maior correção posicional vista neste step (m).
     *
     * @return Valor da maior correção.
     */
    public double maxCorrectionThisStep() {
        return maxCorrectionThisStep;
    }

    /**
     * Valor médio exponencial da velocidade (para debug).
     *
     * @return Média exponencial de |v|.
     */
    public double debugVelAvg() {
        return velAvg;
    }

    /**
     * Temporizador de sono (para debug).
     *
     * @return Valor do temporizador de sono.
     */
    public double debugSleepTimer() {
        return sleepTimer;
    }

    /**
     * Integra o corpo por um passo dt usando Semi-Implicit Euler. Aplica força acumulada + gravidade e um damping
     * linear exponencial leve. Se o corpo estiver dormindo, apenas limpa forças.
     *
     * @param gravity Vetor de gravidade a ser aplicado.
     * @param dt Intervalo de tempo do passo (s).
     */
    public void integrate(Vec3 gravity, double dt) {
        if (isStatic()) {
            clearForces();
            return;
        }
        if (sleeping) {
            clearForces();
            return;
        }

        Vec3 acc = forceAccum.mul(invMass).add(gravity);

        // Semi-Implicit Euler
        velocity = velocity.add(acc.mul(dt));

        // Damping linear (decay exponencial estável)
        double decay = Math.exp(-linearDamping * dt);
        velocity = velocity.mul(decay);

        position = position.add(velocity.mul(dt));
        clearForces();
    }

    /**
     * Atualiza o temporizador de sono com base em: contato, |v| (EMA) e atividade (impulso/correção). Dorme quando:
     * - houve contato (suporte) neste step,
     * - velAvg < vThresholdVel,
     * - maxImpulseThisStep <= impulseQuietThresh e maxCorrectionThisStep <= correctionQuietThresh por tempo contínuo >= timeToSleep.
     *
     * @param dt Intervalo de tempo do passo (s).
     * @param vThresholdVel Limiar de velocidade (m/s) para considerar "parado".
     * @param timeToSleep Tempo contínuo sob o limiar necessário para dormir (s).
     * @param impulseQuietThresh Limiar de impulso para considerar "quieto".
     * @param correctionQuietThresh Limiar de correção posicional para considerar "quieto".
     */
    public void accumulateSleepTimer(
        double dt,
        double vThresholdVel,
        double timeToSleep,
        double impulseQuietThresh,
        double correctionQuietThresh
    ) {
        if (!canSleep || isStatic()) {
            sleepTimer = 0.0;
            sleeping = false;
            velAvg = 0.0;
            return;
        }

        double vLen = velocity.length();
        if (velAvg == 0.0) {
            velAvg = vLen;
        }
        velAvg = EMA_ALPHA * vLen + (1.0 - EMA_ALPHA) * velAvg;

        boolean quietVelocity = velAvg < vThresholdVel;
        boolean quietContacts =
            maxImpulseThisStep <= impulseQuietThresh && maxCorrectionThisStep <= correctionQuietThresh;

        if (hadContactThisStep && quietVelocity && quietContacts) {
            sleepTimer += dt;
            if (sleepTimer >= timeToSleep) {
                sleeping = true;
                velocity = Vec3.ZERO;
            }
        } else {
            sleepTimer = 0.0;
            sleeping = false;
        }
    }


}

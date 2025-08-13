package phys.collision;

import phys.core.RigidBody;
import phys.math.Vec3;

public final class Collision {
    public record Manifold(RigidBody a, RigidBody b, Vec3 normal, double penetration) {
    }

    /**
     * Percentual da penetração a corrigir por passo (0..1).
     */
    public static double POSITION_CORRECTION_PERCENT = 0.95;
    /**
     * “Folga” ignorada na correção de penetração (em metros).
     */
    public static double POSITION_CORRECTION_SLOP = 5e-4;

    // === Contexto de simulação (para atrito em repouso) ===
    private static phys.math.Vec3 GRAVITY = new phys.math.Vec3(0, -9.81, 0);
    private static double CURRENT_DT = 1.0 / 120.0;

    // --- thresholds para acordar (evita acordar por "poeira numérica") ---
    public static double WAKE_IMPULSE_THRESHOLD = 1e-3;     // impulso (Ns) mínimo para acordar
    public static double WAKE_CORRECTION_THRESHOLD = 1e-3;  // correção posicional (m) mínima para acordar

    public static boolean DEBUG_CONTACT = false;

    // Slop para impulso normal: abaixo disso, tratamos como 0 (sem impulso)
    public static double NORMAL_IMPULSE_VSLOP = 2e-3;

    public static int DEBUG_softContactsThisStep = 0;
    public static int DEBUG_resolvesThisStep = 0;


    /**
     * Reseta contadores de debug no início de cada step.
     */
    public static void debugBeginStep() {
        DEBUG_softContactsThisStep = 0;
        DEBUG_resolvesThisStep = 0;
    }

    /**
     * Define o slop de velocidade para impulso normal (m/s).
     */
    public static void setNormalImpulseVSlop(double vslop) {
        NORMAL_IMPULSE_VSLOP = Math.max(0.0, vslop);
    }

    /**
     * Ajusta os limiares de wake. impulse >=0 (Ns), correction >=0 (m)
     */
    public static void setWakeThresholds(double impulse, double correction) {
        WAKE_IMPULSE_THRESHOLD = Math.max(0.0, impulse);
        WAKE_CORRECTION_THRESHOLD = Math.max(0.0, correction);
    }


    /**
     * Define os parâmetros globais de correção posicional.
     *
     * @param percent fração da penetração a corrigir por passo (0..1)
     * @param slop folga ignorada (m)
     */
    public static void setPositionCorrection(double percent, double slop) {
        POSITION_CORRECTION_PERCENT = Math.max(0.0, Math.min(1.0, percent));
        POSITION_CORRECTION_SLOP = Math.max(0.0, slop);
    }

    /**
     * Testa colisão Sphere–Sphere. Convenção: normal aponta de B → A.
     */
    public static Manifold test(RigidBody a, RigidBody b) {
        if (!(a.shape() instanceof Sphere sa) || !(b.shape() instanceof Sphere sb)) {
            return null;
        }

        var pa = a.position();
        var pb = b.position();

        var diff = pb.sub(pa);
        double dist = diff.length();
        double r = sa.radius + sb.radius;

        if (dist >= r) {
            return null;
        }

        // Normal B -> A: invertendo a direção
        Vec3 n = (dist > 0) ? diff.mul(-1.0 / dist) : new Vec3(1, 0, 0);
        double penetration = r - dist;

        return new Manifold(a, b, n, penetration);
    }

    /**
     * Testa colisão Sphere–Plane. Normal do manifold aponta de B → A (plano → esfera). Retorna null quando não há
     * interpenetração “real”.
     * <p>
     * SNAP_SLOP: evita manifold para micro-penetrações — repouso fica a cargo do soft-contact.
     */
    public static Manifold testSpherePlane(RigidBody a, RigidBody b) {
        // Descobrir quem é esfera e quem é plano (a ordem pode vir trocada)
        RigidBody sphereBody;
        RigidBody planeBody;
        boolean swapped = false;

        if (a.shape() instanceof Sphere && b.shape() instanceof Plane) {
            sphereBody = a;
            planeBody = b;
        } else if (b.shape() instanceof Sphere && a.shape() instanceof Plane) {
            sphereBody = b;
            planeBody = a;
            swapped = true;
        } else {
            return null; // tipos não correspondem
        }

        Sphere s = (Sphere) sphereBody.shape();
        Plane p = (Plane) planeBody.shape();

        // Distância do centro da esfera ao plano: n·x - d
        double dist = sphereBody.position().x() * p.normal.x()
                      + sphereBody.position().y() * p.normal.y()
                      + sphereBody.position().z() * p.normal.z() - p.d;

        double penetration = s.radius - dist; // >0 => esfera "para dentro" do plano

        // SNAP_SLOP: não criar manifold para micro-penetrações numéricas
        final double SNAP_SLOP = 1e-3; // 1 mm
        if (penetration <= SNAP_SLOP) {
            return null; // deixa o soft-contact estabilizar (sem impulso normal)
        }

        // Normal do manifold: do plano para a esfera (B -> A)
        Vec3 normal = p.normal;

        // Manter o contrato (A,B) conforme a chamada original
        RigidBody A = sphereBody;
        RigidBody B = planeBody;
        if (swapped) {
            // se b era a esfera, preservamos (a,b) como recebido
            A = a;
            B = b;
            // a normal deve continuar apontando de B->A
            if (A == planeBody && B == sphereBody) {
                normal = p.normal.mul(-1.0);
            } else {
                normal = p.normal;
            }
        }

        return new Manifold(A, B, normal, penetration);
    }


    /**
     * Resolve um contato aplicando: (1) impulso normal (quando há aproximação, com velocity slop), (2) impulso de
     * atrito (Coulomb) no plano tangente, (3) correção posicional (sempre), + estabilização Sphere–Plane.
     * <p>
     * Também marca atividade por passo (contato/impulso/correção) para o sistema de sleep. Convenção: a normal do
     * manifold aponta de B → A.
     */
    public static void resolve(Manifold m) {
        // DEBUG: contou um resolve neste step
        DEBUG_resolvesThisStep++;

        RigidBody a = m.a();
        RigidBody b = m.b();
        Vec3 n = m.normal();

        double invMassSum = a.invMass() + b.invMass();
        if (invMassSum == 0) {
            return;
        }

        // --- marca: houve contato neste step (para o sistema de sleep) ---
        a.markContact();
        b.markContact();

        // (1) Impulso normal (só se aproximando, com velocity slop)
        Vec3 rv = a.velocity().sub(b.velocity());
        double velAlongNormal = rv.dot(n);

        double jNormal = 0.0;
        if (velAlongNormal < -NORMAL_IMPULSE_VSLOP) {
            double e = Math.min(a.restitution(), b.restitution());
            jNormal = -(1.0 + e) * velAlongNormal / invMassSum;

            Vec3 impulseN = n.mul(jNormal);
            if (!a.isStatic()) {
                a.setVelocity(a.velocity().add(impulseN.mul(a.invMass())));
            }
            if (!b.isStatic()) {
                b.setVelocity(b.velocity().sub(impulseN.mul(b.invMass())));
            }

            double jMag = Math.abs(jNormal);
            a.accumulateImpulse(jMag);
            b.accumulateImpulse(jMag);

            if (jMag > WAKE_IMPULSE_THRESHOLD) {
                a.wakeUp();
                b.wakeUp();
            }
        }

        // (2) Atrito tangencial (Coulomb)
        rv = a.velocity().sub(b.velocity());
        Vec3 rvN = n.mul(rv.dot(n));
        Vec3 rvT = rv.sub(rvN);
        double rvTLen = rvT.length();

        if (rvTLen > 1e-9) {
            Vec3 t = rvT.mul(1.0 / rvTLen);

            double muS = 0.5 * (a.frictionStatic() + b.frictionStatic());
            double muK = 0.5 * (a.frictionDynamic() + b.frictionDynamic());

            double jtIdeal = -rv.dot(t) / invMassSum;

            double mA = a.invMass() > 0 ? 1.0 / a.invMass() : 0.0;
            double mB = b.invMass() > 0 ? 1.0 / b.invMass() : 0.0;
            double gAlongN = Math.abs(GRAVITY.x() * n.x() + GRAVITY.y() * n.y() + GRAVITY.z() * n.z());
            double jSupport = (mA + mB) * gAlongN * CURRENT_DT;

            double jN_eff = Math.abs(jNormal) + jSupport;
            double maxStatic = muS * jN_eff;

            if (Math.abs(jtIdeal) <= maxStatic) {
                // ATRITO ESTÁTICO — aplica, mas NÃO acorda
                Vec3 impulseT = t.mul(jtIdeal);
                if (!a.isStatic()) {
                    a.setVelocity(a.velocity().add(impulseT.mul(a.invMass())));
                }
                if (!b.isStatic()) {
                    b.setVelocity(b.velocity().sub(impulseT.mul(b.invMass())));
                }

                double jtMag = Math.abs(jtIdeal);
                a.accumulateImpulse(jtMag);
                b.accumulateImpulse(jtMag);
            } else {
                // ATRITO DINÂMICO — sempre oposto a t; pode acordar
                Vec3 impulseT = t.mul(-muK * jN_eff);
                if (!a.isStatic()) {
                    a.setVelocity(a.velocity().add(impulseT.mul(a.invMass())));
                }
                if (!b.isStatic()) {
                    b.setVelocity(b.velocity().sub(impulseT.mul(b.invMass())));
                }

                double jtMag = muK * jN_eff;
                a.accumulateImpulse(jtMag);
                b.accumulateImpulse(jtMag);

                if (jtMag > WAKE_IMPULSE_THRESHOLD) {
                    a.wakeUp();
                    b.wakeUp();
                }
            }
        }

        // (3) Correção posicional (sempre)
        double percent = POSITION_CORRECTION_PERCENT;
        double slop = POSITION_CORRECTION_SLOP;
        double corrMag = Math.max(m.penetration() - slop, 0.0) / invMassSum * percent;

        a.accumulateCorrection(corrMag);
        b.accumulateCorrection(corrMag);

        if (corrMag > WAKE_CORRECTION_THRESHOLD) {
            a.wakeUp();
            b.wakeUp();
        }

        Vec3 correction = n.mul(corrMag);
        if (!a.isStatic()) {
            a.setPosition(a.position().add(correction.mul(a.invMass())));
        }
        if (!b.isStatic()) {
            b.setPosition(b.position().sub(correction.mul(b.invMass())));
        }

        // (4) Estabilização Sphere–Plane
        boolean aSphere_bPlane = (a.shape() instanceof Sphere) && (b.shape() instanceof Plane);
        boolean bSphere_aPlane = (b.shape() instanceof Sphere) && (a.shape() instanceof Plane);
        if (aSphere_bPlane) {
            stabilizeSpherePlaneContact(a, b);
        } else if (bSphere_aPlane) {
            stabilizeSpherePlaneContact(b, a);
        }
    }


    /**
     * Estabiliza contato de repouso Sphere–Plane:
     * <li>Projeta o centro da esfera para a superfície quando o erro é pequeno (snap)</li>
     * <li>Zera QUALQUER componente de velocidade normal pequena</li>
     * <li>Marca contato e acumula "correção" mesmo sem penetração (importante para sleep). Não acorda os corpos, serve
     * para “colar” levemente o repouso.</li>
     */
    private static void stabilizeSpherePlaneContact(RigidBody sphereBody, RigidBody planeBody) {
        Sphere s = (Sphere) sphereBody.shape();
        Plane p = (Plane) planeBody.shape();

        // distância do centro ao plano: n·x - d
        double dist = sphereBody.position().x() * p.normal.x()
                      + sphereBody.position().y() * p.normal.y()
                      + sphereBody.position().z() * p.normal.z() - p.d;

        double err = s.radius - dist; // >0 se "para dentro" do plano (centro abaixo da superfície)
        final double posEps = 1e-3;   // snap curtinho
        final double velEps = 2e-3;   // zera vN muito pequena

        // Sempre marcar que houve contato "de suporte" neste step
        sphereBody.markContact();
        planeBody.markContact();

        double correctionApplied = 0.0;

        // 1) snap posicional suave (sem acordar)
        if (Math.abs(err) < posEps) {
            sphereBody.setPosition(sphereBody.position().add(p.normal.mul(err)));
            correctionApplied = Math.abs(err);
        }

        // 2) remove componente normal pequena (qualquer sinal) para parar respiração
        double vN = sphereBody.velocity().dot(p.normal);
        if (Math.abs(vN) < velEps) {
            sphereBody.setVelocity(sphereBody.velocity().sub(p.normal.mul(vN)));
        }

        // 3) contabiliza atividade (para o sistema de sleep)
        if (correctionApplied > 0.0) {
            sphereBody.accumulateCorrection(correctionApplied);
            planeBody.accumulateCorrection(correctionApplied);
        }
    }

    /**
     * Contato “suave” Sphere–Plane SEM manifold: usado quando não há penetração, mas a esfera está a uma tolerância da
     * superfície. Marca contato, faz snap posicional, ZERA a componente normal da velocidade e aplica ATRITO TANGENCIAL
     * (Coulomb) usando força normal de suporte ~ m*g. Nunca acorda em atrito estático; pode acordar em atrito
     * dinâmico.
     *
     * @return true se aplicou estabilização de proximidade
     */
    public static boolean softContactSpherePlane(RigidBody sphereBody, RigidBody planeBody) {
        if (!(sphereBody.shape() instanceof Sphere s) || !(planeBody.shape() instanceof Plane p)) {
            return false;
        }

        // distância centro→plano
        double dist = sphereBody.position().x() * p.normal.x()
                      + sphereBody.position().y() * p.normal.y()
                      + sphereBody.position().z() * p.normal.z() - p.d;

        // tolerância para considerar "em contato" sem manifold
        final double contactEps = 5e-3; // 5 mm
        double gap = Math.abs(s.radius - dist);
        if (gap > contactEps) {
            return false;
        }

        // ---- contato presente neste step ----
        sphereBody.markContact();
        planeBody.markContact();

        // SNAP posicional: coloca exatamente apoiado
        double corr = (s.radius - dist);
        if (corr != 0.0) {
            sphereBody.setPosition(sphereBody.position().add(p.normal.mul(corr)));
            double cMag = Math.abs(corr);
            sphereBody.accumulateCorrection(cMag);
            planeBody.accumulateCorrection(cMag);
        }

        // Remove SEMPRE a componente normal de velocidade (elimina “respiração”)
        double vN = sphereBody.velocity().dot(p.normal);
        if (vN != 0.0) {
            sphereBody.setVelocity(sphereBody.velocity().sub(p.normal.mul(vN)));
        }

        // ======= ATRITO TANGENCIAL (Coulomb) =========
        // relative velocity no plano (plano é estático)
        Vec3 rv = sphereBody.velocity(); // planeBody.velocity() == 0
        // separa normal e tangencial
        Vec3 rvN = p.normal.mul(rv.dot(p.normal));
        Vec3 rvT = rv.sub(rvN);
        double rvTLen = rvT.length();

        if (rvTLen > 1e-9) {
            Vec3 t = rvT.mul(1.0 / rvTLen);

            // parâmetros de atrito (média com o plano)
            double muS = 0.5 * (sphereBody.frictionStatic() + planeBody.frictionStatic());
            double muK = 0.5 * (sphereBody.frictionDynamic() + planeBody.frictionDynamic());

            // massas e suporte normal ~ (mA + mB)*|g·n|*dt
            double invMassSum = sphereBody.invMass() + planeBody.invMass(); // tipicamente 1/m + 0
            if (invMassSum > 0.0) {
                double mA = sphereBody.invMass() > 0 ? 1.0 / sphereBody.invMass() : 0.0;
                double mB = planeBody.invMass() > 0 ? 1.0 / planeBody.invMass() : 0.0;

                double gAlongN = Math.abs(GRAVITY.x() * p.normal.x()
                                          + GRAVITY.y() * p.normal.y()
                                          + GRAVITY.z() * p.normal.z());

                double jSupport = (mA + mB) * gAlongN * CURRENT_DT; // impulso normal de suporte

                // impulso ideal para cancelar tangencial
                double jtIdeal = -rv.dot(t) / invMassSum;

                // limite estático e impulso dinâmico
                double jN_eff = jSupport; // não há jNormal aqui
                double maxStatic = muS * jN_eff;

                if (Math.abs(jtIdeal) <= maxStatic) {
                    // ATRITO ESTÁTICO → cancela rv_t completamente (não acorda)
                    Vec3 impulseT = t.mul(jtIdeal);
                    if (!sphereBody.isStatic()) {
                        sphereBody.setVelocity(sphereBody.velocity().add(impulseT.mul(sphereBody.invMass())));
                    }
                    double jtMag = Math.abs(jtIdeal);
                    sphereBody.accumulateImpulse(jtMag);
                    planeBody.accumulateImpulse(jtMag);
                } else {
                    // ATRITO DINÂMICO → magnitude muK * jN_eff, sentido oposto a t (pode acordar)
                    Vec3 impulseT = t.mul(-muK * jN_eff);
                    if (!sphereBody.isStatic()) {
                        sphereBody.setVelocity(sphereBody.velocity().add(impulseT.mul(sphereBody.invMass())));
                    }
                    double jtMag = muK * jN_eff;
                    sphereBody.accumulateImpulse(jtMag);
                    planeBody.accumulateImpulse(jtMag);

                    if (jtMag > WAKE_IMPULSE_THRESHOLD) {
                        sphereBody.wakeUp();
                        planeBody.wakeUp();
                    }
                }
            }
        }

        // DEBUG
        DEBUG_softContactsThisStep++;
        return true;
    }


    /**
     * Testa colisão Sphere–AABB. Convenção: normal aponta de B → A (do AABB para a esfera). Manifold: (a=sphere,
     * b=box).
     */
    public static Manifold testSphereAABB(RigidBody sphereBody, RigidBody boxBody) {
        if (!(sphereBody.shape() instanceof Sphere s) || !(boxBody.shape() instanceof AABB box)) {
            return null;
        }

        var c = sphereBody.position(); // centro da esfera (A)
        var bCenter = boxBody.position(); // centro da caixa (B)
        var bMin = box.min(bCenter);
        var bMax = box.max(bCenter);

        // Ponto mais próximo no AABB ao centro da esfera
        double qx = Math.max(bMin.x(), Math.min(c.x(), bMax.x()));
        double qy = Math.max(bMin.y(), Math.min(c.y(), bMax.y()));
        double qz = Math.max(bMin.z(), Math.min(c.z(), bMax.z()));
        var closest = new Vec3(qx, qy, qz);

        var toBox = closest.sub(c); // A -> B
        double dist = toBox.length();

        // Se o centro está dentro do AABB (dist == 0): escolha normal pela face mais próxima
        if (dist == 0.0) {
            double dxMin = Math.abs(c.x() - bMin.x());
            double dxMax = Math.abs(bMax.x() - c.x());
            double dyMin = Math.abs(c.y() - bMin.y());
            double dyMax = Math.abs(bMax.y() - c.y());
            double dzMin = Math.abs(c.z() - bMin.z());
            double dzMax = Math.abs(bMax.z() - c.z());

            // A normal deve ser B -> A (da caixa para a esfera).
            // Portanto, apontamos para dentro do volume, em direção ao centro da esfera.
            // Faces: se o centro está mais perto do lado MIN em X, normal é +X (da caixa para dentro).
            double min = dxMin;
            Vec3 n = new Vec3(1, 0, 0); // +X (B->A)
            double pen = s.radius + dxMin;

            if (dxMax < min) {
                min = dxMax;
                n = new Vec3(-1, 0, 0);
                pen = s.radius + dxMax;
            }
            if (dyMin < min) {
                min = dyMin;
                n = new Vec3(0, 1, 0);
                pen = s.radius + dyMin;
            }
            if (dyMax < min) {
                min = dyMax;
                n = new Vec3(0, -1, 0);
                pen = s.radius + dyMax;
            }
            if (dzMin < min) {
                min = dzMin;
                n = new Vec3(0, 0, 1);
                pen = s.radius + dzMin;
            }
            if (dzMax < min) { /*min = dzMax;*/
                n = new Vec3(0, 0, -1);
                pen = s.radius + dzMax;
            }

            return new Manifold(sphereBody, boxBody, n, pen);
        }

        // Fora ou tocando: colisão se dist < raio
        if (dist >= s.radius) {
            return null;
        }

        // Normal B -> A: do AABB (ponto mais próximo) para a esfera (centro)
        Vec3 n = (c.sub(closest)).mul(1.0 / dist);
        double penetration = s.radius - dist;

        return new Manifold(sphereBody, boxBody, n, penetration);
    }

    /**
     * Testa colisão AABB–Plane. Manifold: (a=box, b=plane). A normal aponta do PLANO para a CAIXA (usa n do plano).
     */
    public static Manifold testAABBPlane(RigidBody boxBody, RigidBody planeBody) {
        if (!(boxBody.shape() instanceof AABB box) || !(planeBody.shape() instanceof Plane p)) {
            return null;
        }

        var c = boxBody.position();
        // "raio" do AABB ao longo da normal (projeção de meia-extensão na direção da normal)
        double r = Math.abs(box.halfExtents.x() * p.normal.x())
                   + Math.abs(box.halfExtents.y() * p.normal.y())
                   + Math.abs(box.halfExtents.z() * p.normal.z());

        double dist = c.x() * p.normal.x() + c.y() * p.normal.y() + c.z() * p.normal.z() - p.d;

        double penetration = r - dist;
        if (penetration <= 0) {
            return null;
        }

        // normal do manifold usa a normal do plano (aponta do plano para o box)
        return new Manifold(boxBody, planeBody, p.normal, penetration);
    }

    /**
     * Testa colisão AABB–AABB (caixas alinhadas). Convenção: normal aponta de B → A. A penetração é a menor
     * sobreposição entre x, y, z.
     */
    public static Manifold testAABBAABB(RigidBody aBody, RigidBody bBody) {
        if (!(aBody.shape() instanceof AABB a) || !(bBody.shape() instanceof AABB b)) {
            return null;
        }

        var ca = aBody.position(); // A
        var cb = bBody.position(); // B

        double dx = cb.x() - ca.x(); // A->B
        double dy = cb.y() - ca.y();
        double dz = cb.z() - ca.z();

        double ox = (a.halfExtents.x() + b.halfExtents.x()) - Math.abs(dx);
        if (ox <= 0) {
            return null;
        }

        double oy = (a.halfExtents.y() + b.halfExtents.y()) - Math.abs(dy);
        if (oy <= 0) {
            return null;
        }

        double oz = (a.halfExtents.z() + b.halfExtents.z()) - Math.abs(dz);
        if (oz <= 0) {
            return null;
        }

        // escolha do eixo de menor penetração: rastreie o eixo explicitamente
        double penetration = ox;
        int axis = 0; // 0=x, 1=y, 2=z

        if (oy < penetration) {
            penetration = oy;
            axis = 1;
        }
        if (oz < penetration) {
            penetration = oz;
            axis = 2;
        }

        Vec3 n; // B -> A (logo, sinal oposto a A->B)
        if (axis == 0) {
            double sign = Math.signum(dx);
            if (sign == 0) {
                sign = 1;
            }
            n = new Vec3(-sign, 0, 0); // inverte para B->A
        } else if (axis == 1) {
            double sign = Math.signum(dy);
            if (sign == 0) {
                sign = 1;
            }
            n = new Vec3(0, -sign, 0); // B->A
        } else {
            double sign = Math.signum(dz);
            if (sign == 0) {
                sign = 1;
            }
            n = new Vec3(0, 0, -sign); // B->A
        }

        return new Manifold(aBody, bBody, n, penetration);
    }

    /**
     * Define o vetor gravidade global usado para estimar força normal de suporte.
     */
    public static void setGravity(phys.math.Vec3 g) {
        GRAVITY = g;
    }

    /**
     * Define o dt do substep atual (usado na estimativa de impulso normal de suporte).
     */
    public static void setCurrentDt(double dt) {
        CURRENT_DT = Math.max(1e-8, dt);
    }


}

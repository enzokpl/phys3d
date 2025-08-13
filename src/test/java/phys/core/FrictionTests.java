package phys.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phys.math.Vec3;

/**
 * Testes simples para validar atrito por impulso tangencial.
 */
class FrictionTests {

    @Test
    void boxSlowsDownOnGroundDueToFriction() {
        var world = new World();
        world.setFixedTimeStep(1.0 / 120.0);
        world.setSubsteps(4);
        world.setSolverIterations(6);

        var ground = World.staticPlane(new Vec3(0, 1, 0), 0.0);
        var box = World.dynamicBox(new Vec3(0, 0.2, 0), new Vec3(0.3, 0.2, 0.3), 2.0);
        box.setRestitution(0.0);
        box.setFrictionStatic(0.8);
        box.setFrictionDynamic(0.6);
        box.setLinearDamping(0.01);

        // velocidade lateral inicial
        box.setVelocity(new Vec3(3.0, 0.0, 0.0));

        world.addBody(ground);
        world.addBody(box);

        double simTime = 4.0;
        double renderDt = 1.0 / 60.0;

        for (int i = 0; i < (int) Math.round(simTime / renderDt); i++) {
            world.update(renderDt);
        }

        System.out.printf("Box final vel=%.6f,%.6f,%.6f pos=%.6f%n",
            box.velocity().x(), box.velocity().y(), box.velocity().z(), box.position().x());

        // Deve ter praticamente parado no eixo X
        assertTrue(Math.abs(box.velocity().x()) < 0.05, "Atrito não freou o suficiente em X");
        // Deve continuar apoiada no chão (sem afundar)
        assertTrue(box.position().y() >= 0.2 - 1e-3, "Box afundou no chão");
    }

    @Test
    void sphereSlidesFartherWithLowFriction() {
        var worldHi = new World();
        worldHi.setFixedTimeStep(1.0 / 120.0);
        worldHi.setSubsteps(4);

        var worldLo = new World();
        worldLo.setFixedTimeStep(1.0 / 120.0);
        worldLo.setSubsteps(4);

        var ground1 = World.staticPlane(new Vec3(0, 1, 0), 0.0);
        var ground2 = World.staticPlane(new Vec3(0, 1, 0), 0.0);

        var s1 = World.dynamicSphere(new Vec3(0, 0.25, 0), 0.25, 1.0);
        var s2 = World.dynamicSphere(new Vec3(0, 0.25, 0), 0.25, 1.0);

        // alta fricção
        s1.setFrictionStatic(0.9);
        s1.setFrictionDynamic(0.8);
        // baixa fricção
        s2.setFrictionStatic(0.05);
        s2.setFrictionDynamic(0.02);

        // sem damping para evidenciar a diferença de fricção
        s1.setLinearDamping(0.0);
        s2.setLinearDamping(0.0);

        // velocidade lateral inicial (mais alta)
        s1.setVelocity(new Vec3(6.0, 0.0, 0.0));
        s2.setVelocity(new Vec3(6.0, 0.0, 0.0));

        worldHi.addBody(ground1);
        worldHi.addBody(s1);

        worldLo.addBody(ground2);
        worldLo.addBody(s2);

        double simTime = 4.0;         // segundos
        double renderDt = 1.0 / 60.0; // "frames" de jogo

        // medição de "tempo para parar" (primeira vez que |velX| < eps)
        double stopThreshold = 0.05;
        Double tStopHi = null, tStopLo = null;

        double t = 0.0;
        for (int i = 0; i < (int)Math.round(simTime / renderDt); i++) {
            worldHi.update(renderDt);
            worldLo.update(renderDt);

            if (tStopHi == null && Math.abs(s1.velocity().x()) < stopThreshold) tStopHi = t;
            if (tStopLo == null && Math.abs(s2.velocity().x()) < stopThreshold) tStopLo = t;

            t += renderDt;
        }

        System.out.printf("Sphere HiFric  x=%.3f, velX=%.3f, tStop=%.2fs | LoFric x=%.3f, velX=%.3f, tStop=%.2fs%n",
            s1.position().x(), s1.velocity().x(), (tStopHi==null? -1:tStopHi),
            s2.position().x(), s2.velocity().x(), (tStopLo==null? -1:tStopLo));

        // Distância: baixa fricção deve ter andado bem mais
        assertTrue(s2.position().x() > s1.position().x() + 0.25,
            "Baixa fricção não deslizou o esperado (distância).");

        // Tempo para parar: baixa fricção deve demorar mais que alta fricção
        // (se alguma não parou até o fim, tratamos isso como "parou depois")
        double hi = (tStopHi == null) ? simTime : tStopHi;
        double lo = (tStopLo == null) ? simTime : tStopLo;
        assertTrue(lo > hi + 0.3, "Baixa fricção não demorou mais para parar.");
    }

}

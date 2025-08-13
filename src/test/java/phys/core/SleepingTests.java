package phys.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phys.collision.Collision;
import phys.math.Vec3;

/**
 * Testa que um corpo entra em sleep em repouso e acorda ao receber impulso/força.
 * Após acordar, reduzimos o atrito (esfera+plano) para garantir deslocamento > 0.5 m.
 */
class SleepingTests {

    @Test
    void sphereSleepsOnGroundAndWakesWhenPushed() {
        var world = new World();

        // Parâmetros "normais"
        Collision.setNormalImpulseVSlop(2e-3);
        Collision.setWakeThresholds(1e-3, 1e-3);
        Collision.setPositionCorrection(0.95, 5e-4);

        world.setFixedTimeStep(1.0 / 120.0);
        world.setSubsteps(4);
        world.setSolverIterations(6);
        world.setSleepVelThreshold(0.03);
        world.setSleepTime(0.4);

        var ground = World.staticPlane(new Vec3(0,1,0), 0.0);
        var ball = World.dynamicSphere(new Vec3(0, 1.5, 0), 0.25, 1.0);
        ball.setRestitution(0.2);
        ball.setLinearDamping(0.02);

        // Atrito "normal" enquanto cai e repousa
        // (use os nomes de métodos que você tem no RigidBody; assumindo estes:)
        ground.setFrictionStatic(0.6);
        ground.setFrictionDynamic(0.5);
        ball.setFrictionStatic(0.6);
        ball.setFrictionDynamic(0.5);

        world.addBody(ground);
        world.addBody(ball);

        // Simula até dormir (máx 5s)
        double t = 0, renderDt = 1.0/60.0;
        boolean slept = false;
        for (int i = 0; i < (int)Math.round(5.0 / renderDt); i++) {
            world.update(renderDt);
            if (ball.isSleeping()) { slept = true; t = i * renderDt; break; }
        }

        System.out.printf("Dormiu em t=%.2fs? %s, y=%.3f%n", t, slept, ball.position().y());
        assertTrue(slept, "A esfera não entrou em sleep como esperado");

        // ===== Fase do empurrão =====
        // Para garantir deslocamento visível, reduzimos atrito esfera+chão:
        ground.setFrictionStatic(0.2);
        ground.setFrictionDynamic(0.10);
        ball.setFrictionStatic(0.2);
        ball.setFrictionDynamic(0.10);

        // Acorda e dá um impulso tangencial
        ball.wakeUp();
        ball.setVelocity(new Vec3(1.5, 0.0, 0.0));

        double x0 = ball.position().x();
        for (int i = 0; i < 120; i++) { // ~2s (com fixed 120Hz x4 substeps)
            world.update(renderDt);
        }
        double dx = ball.position().x() - x0;

        System.out.printf("Após empurrão: dx=%.3f, sleeping=%s%n", dx, ball.isSleeping());
        assertTrue(dx > 0.5, "A esfera não se moveu o suficiente após acordar");
    }
}

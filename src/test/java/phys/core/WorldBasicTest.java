package phys.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phys.math.Vec3;

class WorldBasicTest {

    @Test
    void sphereFallsAndBouncesOnGround() {
        var world = new World();
        var ground = World.staticPlane(new Vec3(0, 1, 0), 0.0); // y=0
        var ball = World.dynamicSphere(new Vec3(0, 2, 0), 0.25, 1.0);
        ball.setRestitution(0.5);

        world.addBody(ground);
        world.addBody(ball);

        double dt = 1.0 / 120.0; // 120 Hz
        double minY = Double.POSITIVE_INFINITY;
        double maxPenetration = 0.0;
        for (int i = 0; i < 600; i++) { // 5 segundos
            world.step(dt);
            double y = ball.position().y();
            minY = Math.min(minY, y);
            double penetration = Math.max(0, 0.25 - y);
            maxPenetration = Math.max(maxPenetration, penetration);
        }
        System.out.printf("Menor altura da bola: %.6f\n", minY);
        System.out.printf("Maior penetração: %.6f\n", maxPenetration);
        System.out.printf("Altura final: %.6f\n", ball.position().y());
        // deve estar acima ou muito próximo do chão sem penetrar significativamente
        assertTrue(ball.position().y() >= 0.25 - 1e-3, "A bola penetrou o chão mais do que o tolerado.");
        assertTrue(maxPenetration < 1e-2, "Penetração máxima muito alta: " + maxPenetration);
    }
}

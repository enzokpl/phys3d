package phys.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phys.math.Vec3;

/**
 * Verifica se uma AABB cai e repousa corretamente sobre um plano.
 */
class WorldBoxGroundTest {

    @Test
    void boxFallsAndRestsOnGround() {
        var world = new World();
        world.setFixedTimeStep(1.0 / 120.0);
        world.setSubsteps(4);

        var ground = World.staticPlane(new Vec3(0, 1, 0), 0.0); // y=0
        var hx = 0.3;
        var hy = 0.2;
        var hz = 0.25;
        var box = World.dynamicBox(new Vec3(0, 2, 0), new Vec3(hx, hy, hz), 2.0);
        box.setRestitution(0.3);

        world.addBody(ground);
        world.addBody(box);

        double simTime = 5.0;
        double renderDt = 1.0 / 60.0;
        double minY = Double.POSITIVE_INFINITY;
        double maxPen = 0.0;
        for (int i = 0; i < (int) Math.round(simTime / renderDt); i++) {
            world.update(renderDt);
            double y = box.position().y();
            minY = Math.min(minY, y);
            // penetração em relação ao plano y=0: topo inferior do box deve ficar em y=hy
            double penetration = Math.max(0.0, hy - y);
            maxPen = Math.max(maxPen, penetration);
        }

        System.out.printf("Box minY=%.6f, maxPen=%.6f, finalY=%.6f%n", minY, maxPen, box.position().y());

        // altura final deve ser próxima a hy (sem penetrar além do tolerado)
        assertTrue(box.position().y() >= hy - 1e-3, "Caixa penetrou além do tolerado");
        assertTrue(maxPen < 2e-3, "Penetração máxima muito alta: " + maxPen);
    }
}

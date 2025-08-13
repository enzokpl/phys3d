package phys.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phys.math.Vec3;

/**
 * Empilha duas AABBs sobre o plano e verifica estabilidade.
 */
class WorldBoxStackTest {

    @Test
    void twoBoxesStackStableOnGround() {
        var world = new World();
        world.setFixedTimeStep(1.0 / 120.0);
        world.setSubsteps(6);
        world.setSolverIterations(8); // um pouco mais para empilhamento mais "duro"

        var ground = World.staticPlane(new Vec3(0, 1, 0), 0.0); // y=0

        double hy1 = 0.2, hy2 = 0.15;
        var bottom = World.dynamicBox(new Vec3(0, 1.5, 0), new Vec3(0.3, hy1, 0.25), 2.0);
        bottom.setRestitution(0.2);
        var top = World.dynamicBox(new Vec3(0.02, 2.2, 0), new Vec3(0.25, hy2, 0.25), 1.5); // leve offset em x
        top.setRestitution(0.2);

        world.addBody(ground);
        world.addBody(bottom);
        world.addBody(top);

        double simTime = 6.0;
        double renderDt = 1.0 / 60.0;

        double maxPenBottomGround = 0.0;
        double maxPenTopBottom = 0.0;

        for (int i = 0; i < (int) Math.round(simTime / renderDt); i++) {
            world.update(renderDt);

            // pen com o chão: bottom deve ficar com centro em y ~ hy1
            double pb = Math.max(0.0, hy1 - bottom.position().y());
            maxPenBottomGround = Math.max(maxPenBottomGround, pb);

            // pen entre caixas: topo deve ficar "apoiado" em bottom
            // contato nominal quando (top.center.y - hy2) == (bottom.center.y + hy1)
            double desiredTopY = bottom.position().y() + hy1 + hy2;
            double pt = Math.max(0.0, desiredTopY - top.position().y());
            maxPenTopBottom = Math.max(maxPenTopBottom, pt);
        }

        System.out.printf("Bottom y=%.6f, Top y=%.6f%n", bottom.position().y(), top.position().y());
        System.out.printf("maxPen bottom-ground=%.6f, top-bottom=%.6f%n", maxPenBottomGround, maxPenTopBottom);

        // tolerâncias razoáveis com 4 substeps e 6 iterações
        assertTrue(bottom.position().y() >= hy1 - 1e-3, "Bottom penetrou o chão além do tolerado");
        assertTrue(top.position().y() >= bottom.position().y() + hy1 + hy2 - 2e-3, "Top penetrou a caixa de baixo");
        assertTrue(maxPenBottomGround < 2e-3, "Penetração (bottom-ground) alta");
        assertTrue(maxPenTopBottom < 3e-3, "Penetração (top-bottom) alta");
    }
}

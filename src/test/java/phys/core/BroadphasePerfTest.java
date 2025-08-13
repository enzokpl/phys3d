package phys.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phys.broadphase.Broadphase;
import phys.broadphase.UniformGridBroadphase;
import phys.math.Vec3;

/**
 * Verifica que a broadphase por grade reduz fortemente o número de pares candidatos em uma cena esparsa (muitos corpos
 * espalhados).
 */
class BroadphasePerfTest {

    @Test
    void uniformGridReducesCandidatePairs() {
        var world = new World();
        world.setFixedTimeStep(1.0 / 120.0);
        world.setSubsteps(1);
        world.setSolverIterations(1);

        // chão
        world.addBody(World.staticPlane(new Vec3(0, 1, 0), 0.0));

        // 100 caixas pequenas espalhadas
        int N = 100;
        for (int i = 0; i < N; i++) {
            double x = (i % 10) * 3.0;
            double z = (i / 10) * 3.0;
            world.addBody(World.dynamicBox(new Vec3(x, 2.0, z), new Vec3(0.25, 0.25, 0.25), 1.0));
        }

        // --- brute-force: contar pares ---
        int brutePairs = countPairsBruteforce(world);

        // --- grid: contar pares ---
        var grid = new UniformGridBroadphase(2.0); // célula ~ duas caixas
        int gridPairs = countPairsWith(world, grid);

        System.out.printf("Brute pairs=%d, Grid pairs=%d%n", brutePairs, gridPairs);

        // grade deve reduzir bem (ordem de magnitude) em cena esparsa
        assertTrue(gridPairs < brutePairs * 0.4, "Grid não reduziu o bastante os pares");
    }

    private int countPairsBruteforce(World w) {
        int n = w.bodies().size();
        return n * (n - 1) / 2;
    }

    private int countPairsWith(World w, Broadphase bp) {
        bp.clear();
        double[] min = new double[3];
        double[] max = new double[3];

        int planes = 0;
        for (var b : w.bodies()) {
            if (b.shape() instanceof phys.collision.Plane) {
                planes++;
                continue;
            }
            if (phys.collision.Bounds.compute(b, min, max)) {
                bp.insert(b, min, max);
            }
        }
        int count = bp.computePairs().size();
        // pares com planos (dinâmicos × planos)
        int dynamics = w.bodies().size() - planes;
        count += dynamics * planes;
        return count;
    }
}

package phys.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import phys.collision.AABB;
import phys.collision.Collision;
import phys.math.Vec3;

/**
 * Demonstra o ganho de substeps em Sphere–AABB quando o slop não limita a penetração.
 */
class WorldFixedStepTest {

    @Test
    void substepsReducePenetrationOnSphereAabbImpact() {
        double radius = 0.25;
        double simTime = 5.0;

        // Remover "piso" de correção: slop bem pequeno para medir penetração de verdade
        Collision.setPositionCorrection(0.95, 1e-5);

        // Baseline: 30 Hz, sem substeps (grosseiro, pior penetração)
        double dtBaseline = 1.0 / 30.0;
        double maxPenBaseline = runBaselineStepSphereAabb(radius, dtBaseline, simTime);

        // Fixed + substeps: 120 Hz * 4 = 480 Hz internos (deve reduzir penetração)
        double fixedTimeStep = 1.0 / 120.0;
        int substeps = 4;
        double renderDt = 1.0 / 60.0;
        double maxPenSubsteps = runFixedUpdateSphereAabb(radius, fixedTimeStep, substeps, renderDt, simTime);

        System.out.printf("Baseline 30Hz: maxPen=%.6f%n", maxPenBaseline);
        System.out.printf("Fixed 120Hz x4: maxPen=%.6f%n", maxPenSubsteps);

        assertTrue(maxPenSubsteps <= maxPenBaseline * 0.95 + 2e-6,
            "Substeps não reduziram o suficiente: baseline=" + maxPenBaseline + " substeps=" + maxPenSubsteps);
        assertTrue(maxPenSubsteps < 2.0e-5,
            "Penetração com substeps ainda alta: " + maxPenSubsteps);
    }

    private double runBaselineStepSphereAabb(double radius, double dt, double simTimeSeconds) {
        World world = new World();

        var ground = new RigidBody(new Vec3(0, 0.5, 0), 0.0, new AABB(new Vec3(50, 0.5, 50))); // topo em y=1.0
        var ball = World.dynamicSphere(new Vec3(0, 6.0, 0), radius, 1.0); // queda maior = impacto mais forte
        ball.setRestitution(0.0);

        world.addBody(ground);
        world.addBody(ball);

        int steps = (int) Math.round(simTimeSeconds / dt);
        double maxPenetration = 0.0;
        for (int i = 0; i < steps; i++) {
            world.step(dt);
            double pen = penetrationSphereAabb(ball.position(), radius, ground);
            if (pen > maxPenetration) maxPenetration = pen;
        }
        return maxPenetration;
    }

    private double runFixedUpdateSphereAabb(double radius, double fixedTimeStep, int substeps, double renderDt, double simTimeSeconds) {
        World world = new World();
        world.setFixedTimeStep(fixedTimeStep);
        world.setSubsteps(substeps);

        var ground = new RigidBody(new Vec3(0, 0.5, 0), 0.0, new AABB(new Vec3(50, 0.5, 50)));
        var ball = World.dynamicSphere(new Vec3(0, 6.0, 0), radius, 1.0);
        ball.setRestitution(0.0);

        world.addBody(ground);
        world.addBody(ball);

        int frames = (int) Math.round(simTimeSeconds / renderDt);
        double maxPenetration = 0.0;
        for (int i = 0; i < frames; i++) {
            world.update(renderDt);
            double pen = penetrationSphereAabb(ball.position(), radius, ground);
            if (pen > maxPenetration) maxPenetration = pen;
        }
        return maxPenetration;
    }

    private double penetrationSphereAabb(Vec3 center, double radius, RigidBody ground) {
        AABB box = (AABB) ground.shape();
        Vec3 bCenter = ground.position();
        Vec3 bMin = box.min(bCenter);
        Vec3 bMax = box.max(bCenter);

        double qx = Math.max(bMin.x(), Math.min(center.x(), bMax.x()));
        double qy = Math.max(bMin.y(), Math.min(center.y(), bMax.y()));
        double qz = Math.max(bMin.z(), Math.min(center.z(), bMax.z()));

        double dx = center.x() - qx;
        double dy = center.y() - qy;
        double dz = center.z() - qz;
        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return Math.max(0.0, radius - dist);
    }
}

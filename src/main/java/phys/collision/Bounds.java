package phys.collision;

import phys.core.RigidBody;
import phys.math.Vec3;

/**
 * Utilitários para AABBs de limites (broadphase). Planos infinitos são ignorados.
 */
public final class Bounds {
    private Bounds() {
    }

    /**
     * Calcula o AABB (min,max) de um corpo. Retorna false para planos infinitos (sem AABB finita).
     *
     * @param body corpo
     * @param min saída [x,y,z]
     * @param max saída [x,y,z]
     * @return true se o AABB foi calculado; false para shapes não limitados (ex.: Plane).
     */
    public static boolean compute(RigidBody body, double[] min, double[] max) {
        if (body.shape() instanceof Sphere s) {
            Vec3 c = body.position();
            double r = s.radius;
            min[0] = c.x() - r;
            min[1] = c.y() - r;
            min[2] = c.z() - r;
            max[0] = c.x() + r;
            max[1] = c.y() + r;
            max[2] = c.z() + r;
            return true;
        }
        if (body.shape() instanceof AABB box) {
            Vec3 c = body.position();
            Vec3 h = box.halfExtents;
            min[0] = c.x() - h.x();
            min[1] = c.y() - h.y();
            min[2] = c.z() - h.z();
            max[0] = c.x() + h.x();
            max[1] = c.y() + h.y();
            max[2] = c.z() + h.z();
            return true;
        }
        if (body.shape() instanceof Plane) {
            return false; // plano é infinito — tratamos à parte
        }
        // default: não conhecido
        return false;
    }
}

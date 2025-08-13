package phys.collision;

import phys.math.Vec3;

/**
 * Caixa alinhada aos eixos (AABB) definida por seus semi-eixos (half-extents). A posição do RigidBody associado é o
 * centro da caixa.
 */
public final class AABB implements Shape {
    public final Vec3 halfExtents; // (hx, hy, hz) > 0

    public AABB(Vec3 halfExtents) {
        this.halfExtents = halfExtents;
    }

    /**
     * Retorna o min corner em world-space dado o centro.
     *
     * @param center Centro da AABB em world-space.
     */
    public Vec3 min(Vec3 center) {
        return new Vec3(center.x() - halfExtents.x(), center.y() - halfExtents.y(), center.z() - halfExtents.z());
    }

    /**
     * Retorna o max corner em world-space dado o centro.
     *
     * @param center Centro da AABB em world-space.
     */
    public Vec3 max(Vec3 center) {
        return new Vec3(center.x() + halfExtents.x(), center.y() + halfExtents.y(), center.z() + halfExtents.z());
    }
}

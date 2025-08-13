package phys.collision;

import phys.math.Vec3;

/**
 * Plano: n·x = d, com n normal unitária. Usaremos como "chão" y=0 (n=(0,1,0), d=0).
 */
public final class Plane implements Shape {
    public final Vec3 normal; // deve ser unitária
    public final double d;    // deslocamento ao longo da normal

    public Plane(Vec3 normal, double d) {
        double len = Math.sqrt(normal.x() * normal.x() + normal.y() * normal.y() + normal.z() * normal.z());
        this.normal = (len == 1.0) ? normal : new Vec3(normal.x() / len, normal.y() / len, normal.z() / len);
        this.d = d;
    }

    public static Plane groundY0() {
        return new Plane(new Vec3(0, 1, 0), 0.0);
    }
}

package phys.collision;

/**
 * Representa uma esfera para fins de detecção de colisão.
 */
public final class Sphere implements Shape {

    /**
     * Raio da esfera.
     */
    public final double radius;

    /**
     * Cria uma nova esfera com o raio especificado.
     *
     * @param radius o raio da esfera
     */
    public Sphere(double radius) {
        this.radius = radius;
    }
}

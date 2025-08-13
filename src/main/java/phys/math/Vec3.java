package phys.math;

/**
 * Representa um vetor 3D imutável com operações básicas de álgebra vetorial.
 *
 * @param x componente X
 * @param y componente Y
 * @param z componente Z
 */
public record Vec3(double x, double y, double z) {

    /**
     * Vetor nulo (0,0,0).
     */
    public static final Vec3 ZERO = new Vec3(0.0, 0.0, 0.0);

    /**
     * Soma este vetor com outro.
     *
     * @param o vetor a ser somado
     * @return novo vetor resultado da soma
     */
    public Vec3 add(Vec3 o) {
        return new Vec3(x + o.x, y + o.y, z + o.z);
    }

    /**
     * Subtrai outro vetor deste.
     *
     * @param o vetor a ser subtraído
     * @return novo vetor resultado da subtração
     */
    public Vec3 sub(Vec3 o) {
        return new Vec3(x - o.x, y - o.y, z - o.z);
    }

    /**
     * Multiplica este vetor por um escalar.
     *
     * @param s escalar
     * @return novo vetor escalado
     */
    public Vec3 mul(double s) {
        return new Vec3(x * s, y * s, z * s);
    }

    /**
     * Produto escalar entre este vetor e outro.
     *
     * @param o outro vetor
     * @return valor do produto escalar
     */
    public double dot(Vec3 o) {
        return x * o.x + y * o.y + z * o.z;
    }

    /**
     * Retorna o comprimento (norma Euclidiana) deste vetor.
     *
     * @return comprimento do vetor
     */
    public double length() {
        return Math.sqrt(dot(this));
    }

    /**
     * Retorna uma cópia normalizada deste vetor (mesma direção, comprimento 1).
     * Se o vetor for nulo, retorna ele mesmo.
     *
     * @return vetor normalizado
     */
    public Vec3 normalized() {
        double len = length();
        return (len == 0) ? this : this.mul(1.0 / len);
    }


}

package phys.broadphase;

import java.util.List;
import phys.core.RigidBody;

/**
 * Broadphase gera pares potenciais de colisão (pruning grosso). Implementações devem ser stateless por atualização
 * (reconstruídas a cada frame) ou limpar estado via clear().
 */
public interface Broadphase {

    /**
     * Limpa qualquer estrutura interna (antes de uma nova construção).
     */
    void clear();

    /**
     * Insere um corpo e seu AABB de limites no broadphase.
     *
     * @param body corpo rígido
     * @param min min corner do AABB em world-space
     * @param max max corner do AABB em world-space
     */
    void insert(RigidBody body, double[] min, double[] max);

    /**
     * Retorna a lista de pares potencialmente colidentes. Pares não devem se repetir e não incluir (a==b).
     */
    List<RigidBodyPair> computePairs();

    /**
     * Tupla simples (a,b).
     */
    record RigidBodyPair(RigidBody a, RigidBody b) {
    }
}

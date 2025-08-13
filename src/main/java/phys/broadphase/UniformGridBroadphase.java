package phys.broadphase;

import java.util.*;
import phys.core.RigidBody;

/**
 * Broadphase por grade uniforme 3D. Cada AABB é mapeado para um conjunto de células. Pares são gerados por célula e
 * deduplicados.
 */
public final class UniformGridBroadphase implements Broadphase {
    private final double cellSize;
    private final Map<Long, List<Entry>> cells = new HashMap<>();
    private final Set<Long> touched = new HashSet<>(); // células usadas no frame

    private static final class Entry {
        final RigidBody body;
        final int minX;
        final int minY;
        final int minZ;
        final int maxX;
        final int maxY;
        final int maxZ;

        Entry(RigidBody b, int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
            this.body = b;
            this.minX = minX;
            this.minY = minY;
            this.minZ = minZ;
            this.maxX = maxX;
            this.maxY = maxY;
            this.maxZ = maxZ;
        }
    }

    /**
     * @param cellSize tamanho da célula (m). Use algo da ordem da média dos objetos.
     */
    public UniformGridBroadphase(double cellSize) {
        this.cellSize = Math.max(1e-6, cellSize);
    }

    @Override
    public void clear() {
        cells.clear();
        touched.clear();
    }

    @Override
    public void insert(RigidBody body, double[] min, double[] max) {
        int minX = floorDiv(min[0]);
        int minY = floorDiv(min[1]);
        int minZ = floorDiv(min[2]);
        int maxX = floorDiv(max[0]);
        int maxY = floorDiv(max[1]);
        int maxZ = floorDiv(max[2]);
        Entry e = new Entry(body, minX, minY, minZ, maxX, maxY, maxZ);
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    long key = mortonKey(x, y, z);
                    cells.computeIfAbsent(key, k -> new ArrayList<>()).add(e);
                    touched.add(key);
                }
            }
        }
    }

    @Override
    public List<RigidBodyPair> computePairs() {
        Set<Long> seenPair = new HashSet<>();
        List<RigidBodyPair> out = new ArrayList<>();

        for (long key : touched) {
            List<Entry> list = cells.get(key);
            if (list == null || list.size() < 2) {
                continue;
            }

            int n = list.size();
            for (int i = 0; i < n; i++) {
                for (int j = i + 1; j < n; j++) {
                    RigidBody a = list.get(i).body;
                    RigidBody b = list.get(j).body;
                    if (a == b) {
                        continue;
                    }
                    long pid = pairKey(a, b);
                    if (seenPair.add(pid)) {
                        out.add(new RigidBodyPair(a, b));
                    }
                }
            }
        }
        return out;
    }

    private int floorDiv(double v) {
        return (int) Math.floor(v / cellSize);
    }

    private static long mortonKey(int x, int y, int z) {
        // compacta 3 ints em um long (evita colisões triviais). Simples hash 3D:
        long A = x * 73856093L;
        long B = y * 19349663L;
        long C = z * 83492791L;
        return A ^ B ^ C;
    }

    private static long pairKey(RigidBody a, RigidBody b) {
        long ia = System.identityHashCode(a);
        long ib = System.identityHashCode(b);
        return (ia < ib) ? (ia << 32) ^ ib : (ib << 32) ^ ia;
    }
}

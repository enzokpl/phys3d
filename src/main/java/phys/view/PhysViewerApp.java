package phys.view;

import java.util.HashMap;
import java.util.Map;
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.geometry.Point3D;
import javafx.scene.*;
import javafx.scene.control.Label;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Shape3D;
import javafx.scene.shape.Sphere;
import javafx.scene.text.Font;
import javafx.scene.transform.Rotate;
import javafx.stage.Stage;
import phys.collision.Plane;
import phys.core.RigidBody;
import phys.core.World;
import phys.math.Vec3;

/**
 * Viewer 3D para inspecionar a simulação física.
 * <p>
 * Funcionalidades:
 * <ul>
 *   <li>Câmera orbital (LMB arrasta), zoom (scroll), pan (Shift + arrastar)</li>
 *   <li>Grid xadrez no chão e eixos XYZ</li>
 *   <li>HUD com FPS/ajuda</li>
 *   <li>Cor dos corpos muda quando entram em sleep</li>
 *   <li>Teclas: R reset | Espaço empurra esfera | G mostra/esconde grid</li>
 * </ul>
 */
public class PhysViewerApp extends Application {

    /**
     * Escala de conversão: 1 metro = 100 pixels.
     */
    private static final double M2PX = 100.0;

    /**
     * Mapeamento entre corpos físicos e seus nós visuais.
     */
    private final Map<RigidBody, Node> nodes = new HashMap<>();

    /**
     * Mundo físico da simulação.
     */
    private World world;

    /**
     * Grupo raiz dos objetos 3D.
     */
    private Group worldGroup;

    /**
     * Câmera principal da cena.
     */
    private PerspectiveCamera cam;

    /**
     * Rotação X da câmera orbital.
     */
    private final Rotate camRX = new Rotate(-30, Rotate.X_AXIS);

    /**
     * Rotação Y da câmera orbital.
     */
    private final Rotate camRY = new Rotate(35, Rotate.Y_AXIS);

    /**
     * Distância da câmera ao centro da cena (em px).
     */
    private double camDist = 900;

    /**
     * Pan da câmera em X.
     */
    private double camTX = 0;

    /**
     * Pan da câmera em Y.
     */
    private double camTY = 80;

    /**
     * Última posição X do mouse.
     */
    private double lastX, lastY;

    /**
     * Indica se está em modo pan.
     */
    private boolean panning = false;

    /**
     * Nó do grid do chão (para alternar visibilidade).
     */
    private Node gridNode;

    /**
     * Label do HUD (FPS e ajuda).
     */
    private Label hud;

    /**
     * Acumulador de tempo para cálculo de FPS.
     */
    private double fpsTime = 0;

    /**
     * Contador de frames para cálculo de FPS.
     */
    private int fpsFrames = 0;

    /**
     * SubScene 3D principal.
     */
    private SubScene sub;

    /**
     * Velocidade inicial (m/s) da esfera arremessada.
     */
    private static final double THROW_SPEED = 6.0;

    /**
     * Raio da esfera arremessada (m).
     */
    private static final double THROW_RADIUS = 0.15;

    /**
     * Massa da esfera arremessada (kg).
     */
    private static final double THROW_MASS = 0.5;

    /**
     * Inicializa a aplicação JavaFX, monta a cena, HUD e interações.
     *
     * @param stage Palco principal da aplicação.
     */
    @Override
    public void start(Stage stage) {
        setupWorld();

        worldGroup = new Group();
        buildSceneNodes();

        // chão + grid + eixos
        gridNode = makeGroundChecker(20, 20, 0.5); // 20x20 quadros de 0.5m
        worldGroup.getChildren().add(gridNode);
        worldGroup.getChildren().add(makeAxes(2.0)); // eixos de 2m

        // luzes
        var key = new PointLight(Color.WHITE);
        key.setTranslateX(600);
        key.setTranslateY(-800);
        key.setTranslateZ(-600);
        var amb = new AmbientLight(Color.color(0.35, 0.35, 0.4));
        worldGroup.getChildren().addAll(key, amb);

        // SubScene 3D
        sub = new SubScene(worldGroup, 1280, 720, true, SceneAntialiasing.BALANCED);
        sub.setFill(Color.web("#0c0f14"));

        cam = new PerspectiveCamera(true);
        cam.setFieldOfView(45);
        cam.setNearClip(0.05);
        cam.setFarClip(20000);
        sub.setCamera(cam);
        updateCameraTransform();

        // HUD
        hud = new Label(helpText("FPS: --.- | Grid: ON"));
        hud.setTextFill(Color.web("#e6eef7"));
        hud.setFont(Font.font("Consolas", 14));
        hud.setPadding(new Insets(8));
        StackPane root = new StackPane(sub, hud);
        StackPane.setAlignment(hud, javafx.geometry.Pos.TOP_LEFT);

        Scene scene = new Scene(root);
        stage.setTitle("Physics 3D Viewer");
        stage.setScene(scene);
        stage.show();

        // ===== Interações =====
        sub.setOnScroll(e -> {
            camDist = clamp(camDist - e.getDeltaY(), 120, 4000);
            updateCameraTransform();
        });
        sub.setOnMousePressed(e -> {
            lastX = e.getSceneX();
            lastY = e.getSceneY();
            panning = e.isShiftDown();
        });
        sub.setOnMouseDragged(e -> {
            double dx = e.getSceneX() - lastX, dy = e.getSceneY() - lastY;
            lastX = e.getSceneX();
            lastY = e.getSceneY();
            if (panning) {
                camTX += dx * (camDist / 800.0);
                camTY += dy * (camDist / 800.0);
            } else {
                camRY.setAngle(camRY.getAngle() - dx * 0.25);
                camRX.setAngle(clamp(camRX.getAngle() + dy * 0.25, -85, 85));
            }
            updateCameraTransform();
        });

        scene.setOnKeyPressed(e -> {
            switch (e.getCode()) {
                case R -> resetWorld();
                case SPACE -> giveSpherePush();
                case G -> toggleGridVisibility();
            }
        });

        // **Clique direito**: arremessar esfera para o ponto clicado
        sub.setOnMouseClicked(e -> {
            if (e.getButton() == javafx.scene.input.MouseButton.SECONDARY) {
                throwSphereFromClick(e); // usa e.getPickResult()
            }
        });

        // loop 60Hz
        AnimationTimer timer = new AnimationTimer() {
            long lastNanos = -1;
            double acc = 0.0;

            @Override
            public void handle(long now) {
                if (lastNanos < 0) {
                    lastNanos = now;
                    return;
                }
                double dt = (now - lastNanos) / 1e9;
                lastNanos = now;

                acc += dt;
                double renderDt = 1.0 / 60.0;
                while (acc >= renderDt) {
                    world.update(renderDt);
                    acc -= renderDt;
                }
                syncNodes();
                updateHud(dt);
            }
        };
        timer.start();
    }

    /**
     * Atualiza a transformação da câmera orbital (posição, rotação, pan, zoom).
     */
    private void updateCameraTransform() {
        Group g = new Group();
        g.getTransforms().setAll(camRY, camRX);
        cam.getTransforms().setAll(g.getTransforms());
        cam.setTranslateX(camTX);
        cam.setTranslateY(camTY);
        cam.setTranslateZ(-camDist);
    }

    // ===== Mundo de física demo =====

    /**
     * Cria e configura o mundo físico de demonstração (chão, esfera, caixa).
     */
    private void setupWorld() {
        world = new World();

        phys.collision.Collision.setPositionCorrection(0.95, 5e-4);
        phys.collision.Collision.setNormalImpulseVSlop(2e-3);
        phys.collision.Collision.setWakeThresholds(1e-3, 1e-3);

        world.setFixedTimeStep(1.0 / 120.0);
        world.setSubsteps(4);
        world.setSolverIterations(8);
        world.setSleepVelThreshold(0.03);
        world.setSleepTime(0.4);

        var ground = World.staticPlane(new Vec3(0, 1, 0), 0.0);
        ground.setFrictionStatic(0.6);
        ground.setFrictionDynamic(0.5);

        var ball = World.dynamicSphere(new Vec3(-0.8, 2.0, 0.0), 0.25, 1.0);
        ball.setRestitution(0.4);
        ball.setFrictionStatic(0.4);
        ball.setFrictionDynamic(0.3);

        var box = World.dynamicBox(new Vec3(0.8, 1.6, 0.0), new Vec3(0.2, 0.2, 0.2), 2.0);
        box.setRestitution(0.2);
        box.setFrictionStatic(0.6);
        box.setFrictionDynamic(0.5);

        world.addBody(ground);
        world.addBody(ball);
        world.addBody(box);
    }

    /**
     * Reinicia o mundo físico e a cena visual.
     */
    private void resetWorld() {
        nodes.clear();
        worldGroup.getChildren().clear();
        setupWorld();
        buildSceneNodes();
        gridNode = makeGroundChecker(20, 20, 0.5);
        worldGroup.getChildren().add(gridNode);
        worldGroup.getChildren().add(makeAxes(2.0));
        var key = new PointLight(Color.WHITE);
        key.setTranslateX(600);
        key.setTranslateY(-800);
        key.setTranslateZ(-600);
        worldGroup.getChildren().addAll(key, new AmbientLight(Color.color(0.35, 0.35, 0.4)));
    }

    /**
     * Aplica um impulso à primeira esfera encontrada no mundo.
     */
    private void giveSpherePush() {
        for (RigidBody b : world.bodies()) {
            if (b.shape() instanceof phys.collision.Sphere) {
                b.wakeUp();
                b.setVelocity(b.velocity().add(new Vec3(3.0, 0, 0)));
                break;
            }
        }
    }

    /**
     * Alterna a visibilidade do grid do chão e atualiza o HUD.
     */
    private void toggleGridVisibility() {
        if (gridNode != null) {
            gridNode.setVisible(!gridNode.isVisible());
            // atualiza HUD imediatamente com estado do grid
            String status = gridNode.isVisible() ? "ON" : "OFF";
            hud.setText(helpText(String.format("FPS: --.- | Grid: %s", status)));
        }
    }

    // ===== Nodes 3D =====

    /**
     * Cria os nós visuais para todos os corpos do mundo físico.
     */
    private void buildSceneNodes() {
        for (RigidBody b : world.bodies()) {
            Node node = createNodeFor(b);
            if (node != null) {
                nodes.put(b, node);
                worldGroup.getChildren().add(node);
            }
        }
    }

    /**
     * Cria o Node JavaFX correspondente ao corpo físico informado.
     *
     * @param b Corpo físico
     * @return Node visual ou null se não suportado
     */
    private Node createNodeFor(RigidBody b) {
        if (b.shape() instanceof Plane p) {
            // slab visual clicável sobre y=0
            Box mesh = new Box(50 * M2PX, 2, 50 * M2PX);
            mesh.setTranslateY((0 - p.d) * M2PX + 1);
            mesh.setMaterial(mat(Color.web("#1b1f2a")));
            // mesh.setMouseTransparent(false); // padrão já é clicável
            return mesh;
        } else if (b.shape() instanceof phys.collision.Sphere s) {
            double r = s.radius * M2PX;
            Sphere mesh = new Sphere(r, 48);
            mesh.setMaterial(mat(Color.web("#3fa7ff")));
            return mesh;
        } else if (b.shape() instanceof phys.collision.AABB aabb) {
            Vec3 he = aabb.halfExtents; // se for getter: aabb.getHalfExtents()
            Box mesh = new Box(2 * he.x() * M2PX, 2 * he.y() * M2PX, 2 * he.z() * M2PX);
            mesh.setMaterial(mat(Color.web("#ff8f3f")));
            return mesh;
        }
        return null;
    }

    /**
     * Sincroniza a posição e cor dos nós visuais com o estado dos corpos físicos.
     */
    private void syncNodes() {
        for (var e : nodes.entrySet()) {
            RigidBody b = e.getKey();
            Node n = e.getValue();

            if (!(b.shape() instanceof Plane)) {
                var p = b.position();
                n.setTranslateX(p.x() * M2PX);
                n.setTranslateY((0 - p.y()) * M2PX);
                n.setTranslateZ(p.z() * M2PX);
            }
            if (n instanceof Shape3D s) {
                PhongMaterial m = (PhongMaterial) s.getMaterial();
                if (b.isSleeping()) {
                    m.setDiffuseColor(m.getDiffuseColor().darker().darker());
                } else {
                    m.setDiffuseColor(m.getDiffuseColor().brighter());
                }
            }
        }
    }

    // ===== Helpers visuais =====

    /**
     * Cria um material Phong com cor base e especular.
     *
     * @param c Cor base
     * @return Material Phong
     */
    private PhongMaterial mat(Color c) {
        PhongMaterial m = new PhongMaterial(c);
        m.setSpecularColor(c.interpolate(Color.WHITE, 0.35));
        return m;
    }

    /**
     * Cria eixos XYZ (X=vermelho, Y=verde, Z=azul) com comprimento especificado.
     *
     * @param lengthMeters Comprimento em metros
     * @return Grupo com os eixos
     */
    private Group makeAxes(double lengthMeters) {
        double L = lengthMeters * M2PX;
        double R = 2.5;

        Cylinder x = new Cylinder(R, L);
        x.setMaterial(mat(Color.RED));
        x.setRotationAxis(Rotate.Z_AXIS);
        x.setRotate(90);
        x.setTranslateX(L / 2);

        Cylinder y = new Cylinder(R, L);
        y.setMaterial(mat(Color.LIMEGREEN));
        y.setTranslateY(-L / 2);

        Cylinder z = new Cylinder(R, L);
        z.setMaterial(mat(Color.CORNFLOWERBLUE));
        z.setRotationAxis(Rotate.X_AXIS);
        z.setRotate(90);
        z.setTranslateZ(L / 2);

        Group g = new Group(x, y, z);
        g.setMouseTransparent(true);
        return g;
    }

    /**
     * Cria um grid xadrez no chão.
     *
     * @param nx Número de quadros em X
     * @param nz Número de quadros em Z
     * @param cellMeters Tamanho de cada quadro (m)
     * @return Grupo com o grid
     */
    private Group makeGroundChecker(int nx, int nz, double cellMeters) {
        double s = cellMeters * M2PX;
        Group g = new Group();
        for (int ix = -nx; ix < nx; ix++) {
            for (int iz = -nz; iz < nz; iz++) {
                Box quad = new Box(s, 1, s);
                quad.setTranslateX(ix * s + s / 2.0);
                quad.setTranslateY(1);
                quad.setTranslateZ(iz * s + s / 2.0);
                boolean even = ((ix + iz) & 1) == 0;
                Color c = even ? Color.web("#141922") : Color.web("#10141b");
                quad.setMaterial(mat(c));
                g.getChildren().add(quad);
            }
        }
        return g;
    }


    // ===== HUD/FPS =====

    /**
     * Atualiza o HUD com FPS e estado do grid.
     *
     * @param dt Delta de tempo desde o último frame
     */
    private void updateHud(double dt) {
        fpsTime += dt;
        fpsFrames++;
        if (fpsTime >= 0.5) {
            double fps = fpsFrames / fpsTime;
            fpsFrames = 0;
            fpsTime = 0;
            String gridState = (gridNode != null && gridNode.isVisible()) ? "ON" : "OFF";
            hud.setText(helpText(String.format("FPS: %.1f | Grid: %s", fps, gridState)));
        }
    }

    /**
     * Retorna o texto de ajuda do HUD, incluindo informações extras.
     *
     * @param extra Texto adicional
     * @return Texto formatado
     */
    private String helpText(String extra) {
        return """
                   Physics 3D Viewer
                   Mouse: arrastar = orbitar | Shift+arrastar = pan | Scroll = zoom
                   Teclas: R = reset | Espaço = empurrar esfera | G = toggle grid
                   """ + extra;
    }

    /**
     * Limita um valor entre mínimo e máximo.
     *
     * @param v Valor
     * @param lo Mínimo
     * @param hi Máximo
     * @return Valor limitado
     */
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * Cria uma esfera e arremessa na direção do ponto clicado (botão direito). Usa o PickResult do MouseEvent; se não
     * houver interseção, mira "para frente" da câmera.
     *
     * @param e Evento de mouse
     */
    private void throwSphereFromClick(MouseEvent e) {
        // origem do raio = posição da câmera em coords de cena
        Point3D camScene = cam.localToScene(0, 0, 0);
        Vec3 originPhys = new Vec3(camScene.getX() / M2PX, -camScene.getY() / M2PX, camScene.getZ() / M2PX);

        // direção
        Vec3 dirPhys;
        PickResult pr = e.getPickResult();
        if (pr != null && pr.getIntersectedPoint() != null) {
            Point3D hit = pr.getIntersectedPoint();
            Vec3 hitPhys = new Vec3(hit.getX() / M2PX, -hit.getY() / M2PX, hit.getZ() / M2PX);
            dirPhys = hitPhys.sub(originPhys).normalized();
        } else {
            // fallback: vetor "para frente" da câmera
            Point3D forwardScene = cam.localToScene(0, 0, -1);
            dirPhys = new Vec3(
                (forwardScene.getX() - camScene.getX()),
                -(forwardScene.getY() - camScene.getY()),
                (forwardScene.getZ() - camScene.getZ())
            ).normalized();
        }

        // spawn 30 cm à frente da câmera
        Vec3 spawn = originPhys.add(dirPhys.mul(0.30));

        // cria corpo físico
        var body = World.dynamicSphere(spawn, THROW_RADIUS, THROW_MASS);
        body.setRestitution(0.3);
        body.setFrictionStatic(0.4);
        body.setFrictionDynamic(0.3);
        body.setVelocity(dirPhys.mul(THROW_SPEED));

        // registra no mundo e cria o node visual
        addBodyAndNode(body);
    }

    /**
     * Adiciona o corpo no mundo e cria o Node JavaFX correspondente.
     *
     * @param b Corpo físico
     */
    private void addBodyAndNode(RigidBody b) {
        world.addBody(b);
        Node node = createNodeFor(b);
        if (node != null) {
            nodes.put(b, node);
            worldGroup.getChildren().add(node);
        }
    }

    /**
     * Método principal. Inicia a aplicação.
     *
     * @param args Argumentos de linha de comando
     */
    public static void main(String[] args) {
        launch(args);
    }
}

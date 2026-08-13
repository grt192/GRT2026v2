package frc.robot.util.subsystems;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.subsystems.MechanismConfig.Visualizer;

/**
 * Builds and drives the default {@code Mechanism2d} for a mechanism.
 *
 * <p>
 * Two shapes cover every mechanism we have: a single ligament for anything that swings to an
 * angle, and a spinning polygon with N spokes for anything that rotates continuously. Dimensions
 * come from {@link Visualizer} in the config, so a subsystem that wants the default shape writes
 * no visualization code at all.
 *
 * <p>
 * A subsystem that needs more can append to {@link #mechanism2d()} or {@link #root()} — the
 * intake pivot draws a moving wall that way.
 */
public class MechanismVisualizer {

    /** Coarse polygons on the RIO where the extra vertices are wasted bandwidth. */
    private static final int POLYGON_SIDES = Constants.CURRENT_MODE == Mode.REAL ? 3 : 20;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d[] driven;
    private final double spokeSpacingDeg;

    MechanismVisualizer(String name, Visualizer spec) {
        mechanism = new LoggedMechanism2d(spec.canvasWidthM(), spec.canvasHeightM());
        root = mechanism.getRoot(rootName(name), spec.rootX(), spec.rootY());

        switch (spec.shape()) {
            case ARM:
                driven = new LoggedMechanismLigament2d[] {
                        root.append(new LoggedMechanismLigament2d(
                            "Arm", spec.lengthM(), 0.0, spec.thicknessPx(), spec.color()))
                };
                spokeSpacingDeg = 0.0;
                break;

            case SPINNER:
                driven = buildSpinner(spec);
                spokeSpacingDeg = 360.0 / Math.max(1, spec.spokes());
                break;

            case NONE:
            default:
                driven = new LoggedMechanismLigament2d[0];
                spokeSpacingDeg = 0.0;
                break;
        }
    }

    /**
     * Spokes radiating from the root, with a polygon outline on the first one so the rotation
     * reads as a wheel rather than a bare line.
     */
    private LoggedMechanismLigament2d[] buildSpinner(Visualizer spec) {
        int spokes = Math.max(1, spec.spokes());
        double radius = spec.lengthM();
        double spacingDeg = 360.0 / spokes;

        LoggedMechanismLigament2d[] built = new LoggedMechanismLigament2d[spokes];
        for (int i = 0; i < spokes; i++) {
            built[i] = root.append(new LoggedMechanismLigament2d(
                "Spoke" + i, radius, i * spacingDeg, spec.thicknessPx(),
                new Color8Bit(Color.kOrange)));
        }

        double edgeLength = 2.0 * radius * Math.sin(Math.PI / POLYGON_SIDES);
        double edgeTurnDeg = 360.0 / POLYGON_SIDES;
        LoggedMechanismLigament2d previous = built[0].append(new LoggedMechanismLigament2d(
            "Edge0", edgeLength, 90.0 + edgeTurnDeg / 2.0, 4.0, spec.color()));
        for (int i = 1; i < POLYGON_SIDES; i++) {
            previous = previous.append(new LoggedMechanismLigament2d(
                "Edge" + i, edgeLength, edgeTurnDeg, 4.0, spec.color()));
        }
        return built;
    }

    /** Mechanism2d root names must be unique per canvas but not across canvases. */
    private static String rootName(String name) {
        int lastSlash = name.lastIndexOf('/');
        return lastSlash < 0 ? name : name.substring(lastSlash + 1);
    }

    /** The canvas, for a subsystem appending its own geometry or for logging. */
    public LoggedMechanism2d mechanism2d() {
        return mechanism;
    }

    /** The driven root, for a subsystem appending its own geometry. */
    public LoggedMechanismRoot2d root() {
        return root;
    }

    void update(double positionRot) {
        double deg = Units.rotationsToDegrees(positionRot);
        for (int i = 0; i < driven.length; i++) {
            driven[i].setAngle(deg + i * spokeSpacingDeg);
        }
    }
}

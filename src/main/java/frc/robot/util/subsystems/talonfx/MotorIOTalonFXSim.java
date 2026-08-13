package frc.robot.util.subsystems.talonfx;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.subsystems.MechanismConfig;
import frc.robot.util.subsystems.MechanismConfig.Sim;
import frc.robot.util.subsystems.MechanismState;

/**
 * The same TalonFX and the same configuration as the real thing, driven against a WPILib plant.
 *
 * <p>
 * Because the closed loop runs on the (simulated) motor controller, simulation exercises the
 * identical control path as the robot rather than a RIO-side stand-in. The plant is chosen from
 * {@link MechanismConfig#sim()}, which also carries the gearbox reduction — deliberately
 * separate from the ratio the Talon applies to reported position, since a mechanism can report
 * rotor rotations while still turning a reduction.
 */
class MotorIOTalonFXSim extends MotorIOTalonFX {

    private static final double LOOP_PERIOD_SECONDS = 0.02;

    private final TalonFXSimState simState;
    private final double mechanicalReduction;

    private final SingleJointedArmSim armSim;
    private final DCMotorSim rotatingMassSim;

    MotorIOTalonFXSim(MechanismConfig config, int canId, String role, LoggedCanivore bus) {
        super(config, canId, role, bus);

        simState = talon.getSimState();
        simState.Orientation = config.simOrientation();
        mechanicalReduction = config.sim().mechanicalReduction();

        Sim spec = config.sim();
        if (spec instanceof Sim.Arm arm) {
            armSim = new SingleJointedArmSim(
                arm.gearbox(),
                arm.mechanicalReduction(),
                arm.moiKgM2(),
                arm.comLengthM(),
                Units.rotationsToRadians(arm.minAngleRot()),
                Units.rotationsToRadians(arm.maxAngleRot()),
                arm.simulateGravity(),
                Units.rotationsToRadians(arm.startAngleRot()));
            rotatingMassSim = null;
        } else if (spec instanceof Sim.RotatingMass mass) {
            armSim = null;
            rotatingMassSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    mass.gearbox(), mass.moiKgM2(), mass.mechanicalReduction()),
                mass.gearbox());
        } else {
            armSim = null;
            rotatingMassSim = null;
        }
    }

    /** Output-shaft state, for a simulated absolute encoder that has no plant of its own. */
    @Override
    MechanismState mechanismState() {
        if (armSim != null) {
            return new MechanismState(
                Units.radiansToRotations(armSim.getAngleRads()),
                Units.radiansToRotations(armSim.getVelocityRadPerSec()));
        }
        if (rotatingMassSim != null) {
            return new MechanismState(
                rotatingMassSim.getAngularPositionRotations(),
                rotatingMassSim.getAngularVelocityRPM() / 60.0);
        }
        return MechanismState.ZERO;
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        if (armSim != null) {
            armSim.setInputVoltage(simState.getMotorVoltage());
            armSim.update(LOOP_PERIOD_SECONDS);
        } else if (rotatingMassSim != null) {
            rotatingMassSim.setInputVoltage(simState.getMotorVoltage());
            rotatingMassSim.update(LOOP_PERIOD_SECONDS);
        }

        MechanismState state = mechanismState();
        simState.setRawRotorPosition(state.positionRot() * mechanicalReduction);
        simState.setRotorVelocity(state.velocityRps() * mechanicalReduction);

        super.updateInputs(inputs);
    }
}

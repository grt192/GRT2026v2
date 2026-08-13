package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;

public class PivotAndRollerIntakeCommand extends ParallelCommandGroup {
    public PivotAndRollerIntakeCommand(PivotSubsystem pivot, IntakeRollerSubsystem roller) {
        super(roller.runRollerIn(), pivot.holdPivotOut());
    }
}

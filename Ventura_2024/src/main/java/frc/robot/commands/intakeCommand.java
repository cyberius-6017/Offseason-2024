package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class intakeCommand extends Command {
    
    private intake intake;
    private Supplier<Double> strengthIntake;

    public intakeCommand(intake intake, Supplier<Double> strengthIntake) {
        this.intake = intake;
        this.strengthIntake = strengthIntake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (strengthIntake.get() > 0.25) {
            intake.startRoll();
        } else {
            intake.stopRoll();
        }
    }
}

package frc.robot.commands.coralalgae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeDevice;

public class IntakeAlgae extends Command {
    private CoralAlgaeDevice coralAlgaeDevice = null;
    private int counter = 0;

    public IntakeAlgae(CoralAlgaeDevice coralAlgaeDevice )
    {
        this.coralAlgaeDevice = coralAlgaeDevice;
        addRequirements( coralAlgaeDevice);
    }
    @Override
    public void initialize() {
        counter = 0;
    }

    @Override
    public void execute() {
        coralAlgaeDevice.runAlgaeIntake();
        coralAlgaeDevice.run();
        counter += 1;
    }

    @Override
    public boolean isFinished() {
        return (coralAlgaeDevice.getBottomAlgaeRPM() < 4800 || coralAlgaeDevice.getTopAlgaeRPM() < 4800) && counter > 100;
    }

    @Override
    public void end(boolean interrupted) {
        coralAlgaeDevice.stop();
    }
}

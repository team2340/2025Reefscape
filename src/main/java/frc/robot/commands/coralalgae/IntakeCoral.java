package frc.robot.commands.coralalgae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeDevice;

public class IntakeCoral extends Command {
    private CoralAlgaeDevice coralAlgaeDevice = null;

    public IntakeCoral(CoralAlgaeDevice coralAlgaeDevice )
    {
        this.coralAlgaeDevice = coralAlgaeDevice;
        addRequirements( coralAlgaeDevice);
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        coralAlgaeDevice.runCoralIntake();
        coralAlgaeDevice.run();
    }

    @Override
    public boolean isFinished() {
        return coralAlgaeDevice.frontSensorHasCoral() && !coralAlgaeDevice.rearSensorHasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        coralAlgaeDevice.stop();
    }
}

package frc.robot.commands.coralalgae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeDevice;
import frc.robot.subsystems.Lights;

public class DeployCoral extends Command {
    private CoralAlgaeDevice coralAlgaeDevice = null;
    private int counter = 0;

    public DeployCoral(CoralAlgaeDevice coralAlgaeDevice )
    {
        this.coralAlgaeDevice = coralAlgaeDevice;
        addRequirements( coralAlgaeDevice);
    }
    @Override
    public void initialize() {
        counter = 0;
        Lights.getInstance().setPattern( Lights.PATTERN.DEPLOYING_CORAL );
    }

    @Override
    public void execute() {
        coralAlgaeDevice.runCoralDeploy();
        coralAlgaeDevice.run();
        counter += 1;
    }

    @Override
    public boolean isFinished() {
        return counter > 150;
    }

    @Override
    public void end(boolean interrupted) {
        coralAlgaeDevice.stop();
        coralAlgaeDevice.setHasPiece( false );
        Lights.getInstance().setDefaultPattern();
    }
}

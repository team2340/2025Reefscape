package frc.robot.commands.coralalgae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeDevice;
import frc.robot.subsystems.Lights;

public class IntakeCoral extends Command {
    private CoralAlgaeDevice coralAlgaeDevice = null;
    private boolean hasCoral = false;
    private int count = 0;

    public IntakeCoral(CoralAlgaeDevice coralAlgaeDevice )
    {
        this.coralAlgaeDevice = coralAlgaeDevice;
        addRequirements( coralAlgaeDevice);
    }
    @Override
    public void initialize() {
        super.initialize();
        count = 0;
        hasCoral = false;
        Lights.getInstance().setPattern( Lights.PATTERN.INTAKE_CORAL );
    }

    @Override
    public void execute() {
        if( !hasCoral )
        {
            coralAlgaeDevice.runCoralIntake();
            coralAlgaeDevice.run();
        }
        else
        {
            coralAlgaeDevice.runCoralIntakeBackwards();
        }


        if( coralAlgaeDevice.frontSensorHasCoral() && !coralAlgaeDevice.rearSensorHasCoral() && !hasCoral )
        {
            hasCoral = true;
        }

        if( hasCoral )
        {
            count += 1;
        }
    }

    @Override
    public boolean isFinished() {
        return hasCoral && count > 10;
    }

    @Override
    public void end(boolean interrupted) {
        coralAlgaeDevice.stop();
        Lights.getInstance().setDefaultPattern();
    }
}

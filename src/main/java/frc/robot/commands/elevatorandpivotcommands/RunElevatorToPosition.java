package frc.robot.commands.elevatorandpivotcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;
import frc.robot.subsystems.Lights;

public class RunElevatorToPosition extends Command {
    private final ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;

    public RunElevatorToPosition( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem ) {
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        addRequirements( elevatorAndPivotSubsystem );
    }

    @Override
    public void initialize() {
        Lights.getInstance().setPattern(Lights.PATTERN.MOVING_ELEVATOR);
        elevatorAndPivotSubsystem.setElevatorToQueuedPosition();
        elevatorAndPivotSubsystem.run();
    }

    @Override
    public void execute() {
        elevatorAndPivotSubsystem.run();
    }

    @Override
    public void end( boolean interrupted ) {
        Lights.getInstance().setDefaultPattern();

        if( interrupted )
        {
            elevatorAndPivotSubsystem.setElevatorPIDControllerSetpoint( elevatorAndPivotSubsystem.getCurrentElevatorEncoderPosition() );
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorAndPivotSubsystem.isElevatorAtSetpoint();
    }
}

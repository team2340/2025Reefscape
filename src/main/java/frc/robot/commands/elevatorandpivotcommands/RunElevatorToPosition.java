package frc.robot.commands.elevatorandpivotcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

public class RunElevatorToPosition extends Command {
    private final ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;

    public RunElevatorToPosition( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem ) {
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        addRequirements( elevatorAndPivotSubsystem );
    }

    @Override
    public void initialize() {
        elevatorAndPivotSubsystem.setElevatorToQueuedPosition();
        elevatorAndPivotSubsystem.run();
    }

    @Override
    public void execute() {
        elevatorAndPivotSubsystem.run();
    }

    @Override
    public void end( boolean interrupted ) {

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

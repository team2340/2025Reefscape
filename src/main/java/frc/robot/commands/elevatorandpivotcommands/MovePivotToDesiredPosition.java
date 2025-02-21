package frc.robot.commands.elevatorandpivotcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

/**
 * Moves the pivot to the desired position
 */
public class MovePivotToDesiredPosition extends Command {
    private ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;

    public MovePivotToDesiredPosition( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem )
    {
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        addRequirements( elevatorAndPivotSubsystem );
    }

    @Override
    public void initialize() {
        elevatorAndPivotSubsystem.setPivotToQueuedPosition();
    }

    @Override
    public void execute() {
        elevatorAndPivotSubsystem.run();
    }

    @Override
    public boolean isFinished() {
        return elevatorAndPivotSubsystem.isPivotAtSetpoint();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

/**
 * Main command that gets called. When called, this will move the pivot out of the way for the elevator,
 * then move the elevator to the desired position, and then move the pivot to the desired position
 */
public class MovePivotAndElevatorToPosition {
    public static Command getCommand( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem )
    {
        return new MovePivotToSafePosition()
                .andThen( new RunElevatorToPosition( elevatorAndPivotSubsystem ) )
                .andThen( new MovePivotToDesiredPosition() );
    }
}

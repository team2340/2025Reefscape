package frc.robot.commands.elevatorandpivotcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

/**
 * Main command that gets called. When called, this will move the pivot out of the way for the elevator,
 * then move the elevator to the desired position, and then move the pivot to the desired position
 */
public class MovePivotAndElevatorToPosition extends SequentialCommandGroup {
    public MovePivotAndElevatorToPosition( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem )
    {
        addCommands(
                new MovePivotToSafePosition( elevatorAndPivotSubsystem ),
                new RunElevatorToPosition( elevatorAndPivotSubsystem ),
                new MovePivotToDesiredPosition( elevatorAndPivotSubsystem )
        );
        //addRequirements( elevatorAndPivotSubsystem );
    }
}

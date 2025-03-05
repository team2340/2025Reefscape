package frc.robot.commands.elevatorandpivotcommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

public class BringElevatorBackDown extends SequentialCommandGroup {
    public BringElevatorBackDown( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem )
    {
        addCommands(
                new InstantCommand( () -> elevatorAndPivotSubsystem.setQueuedElevatorPosition( ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE )),
                new InstantCommand( () -> elevatorAndPivotSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.STOWED )),
                new MovePivotAndElevatorToPosition( elevatorAndPivotSubsystem ) );
        //addRequirements( elevatorAndPivotSubsystem );
    }
}

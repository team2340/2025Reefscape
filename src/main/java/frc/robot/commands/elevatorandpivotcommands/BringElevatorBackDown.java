package frc.robot.commands.elevatorandpivotcommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

import java.util.concurrent.atomic.AtomicReference;

public class BringElevatorBackDown extends SequentialCommandGroup {
    private static AtomicReference<ElevatorAndPivotSubsystem.ElevatorPositions> prevElevator = new AtomicReference<>(ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE);
    private static AtomicReference<ElevatorAndPivotSubsystem.PivotAngles> prevPivot = new AtomicReference<ElevatorAndPivotSubsystem.PivotAngles>(ElevatorAndPivotSubsystem.PivotAngles.STOWED);
    public BringElevatorBackDown( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem )
    {
        addCommands(
                new InstantCommand( () -> {
                    prevElevator.set( elevatorAndPivotSubsystem.getCurrentDesiredElevatorPosition());
                    prevPivot.set(elevatorAndPivotSubsystem.getCurrentPivotDesired());
                }),
                new InstantCommand( () -> elevatorAndPivotSubsystem.setQueuedElevatorPosition( ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE )),
                new InstantCommand( () -> elevatorAndPivotSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.STOWED )),
                new MovePivotAndElevatorToPosition( elevatorAndPivotSubsystem ).finallyDo( () -> {
                    elevatorAndPivotSubsystem.setQueuedElevatorPosition( prevElevator.get() );
                    elevatorAndPivotSubsystem.setQueuedPivotAngle( prevPivot.get() );
                }));
        //addRequirements( elevatorAndPivotSubsystem );
    }
}

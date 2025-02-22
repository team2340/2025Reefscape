package frc.robot.commands.elevatorandpivotcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorAndPivotConstants;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

/**
 * Moves the pivot to a safe position where the elevator can go up and down and not have it hit things
 */
public class MovePivotToSafePosition extends Command {
    private ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;

    public MovePivotToSafePosition( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem )
    {
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        addRequirements( elevatorAndPivotSubsystem );
    }

    @Override
    public void initialize() {
        elevatorAndPivotSubsystem.setPivotPIDControllerSetpoint( ElevatorAndPivotConstants.MINIMUM_PIVOT_ANGLE_FOR_ELEVATOR_MOVEMENT );
        elevatorAndPivotSubsystem.run();

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

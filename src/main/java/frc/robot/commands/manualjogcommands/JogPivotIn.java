package frc.robot.commands.manualjogcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

public class JogPivotIn extends Command {
    private final ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;

    public JogPivotIn( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem ) {
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        addRequirements( elevatorAndPivotSubsystem );
    }

    @Override
    public void execute() {
        elevatorAndPivotSubsystem.pivotIn();
    }

    @Override
    public void end( boolean interrupted ) {
        elevatorAndPivotSubsystem.setPivotPIDControllerSetpoint( elevatorAndPivotSubsystem.getCurrentPivotEncoderPosition() );
        elevatorAndPivotSubsystem.stopPivot();
    }
}

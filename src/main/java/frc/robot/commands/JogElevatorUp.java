package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

public class JogElevatorUp extends Command {
    private final ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;

    public JogElevatorUp( ElevatorAndPivotSubsystem elevatorAndPivotSubsystem ) {
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        addRequirements( elevatorAndPivotSubsystem );
    }

    @Override
    public void execute() {
        elevatorAndPivotSubsystem.startJogElevatorUp();
    }

    @Override
    public void end( boolean interrupted ) {
        elevatorAndPivotSubsystem.setElevatorPIDControllerSetpoint( elevatorAndPivotSubsystem.getCurrentElevatorEncoderPosition() );
        elevatorAndPivotSubsystem.stopElevator();
    }
}

package frc.robot.commands.coralalgae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.swervedrive.AutoDriving;
import frc.robot.subsystems.CoralAlgaeDevice;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

import java.util.Map;

/**
 * This takes in the current setting for the elevator, pivot, auto drive positions and determines what to do with the
 * coral device (deploy, intake, collect coral, deploy algae)
 */
public class RunCoralAlgaeDeviceAutomatic {
    private enum Mode {
        INTAKE_CORAL,
        DEPLOY_CORAL,
        INTAKE_ALGAE,
        DEPLOY_ALGAE
    }

    private AutoDriving driveToReef;
    private ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;
    private CoralAlgaeDevice coralAlgaeDevice;

    public RunCoralAlgaeDeviceAutomatic(AutoDriving driveToReef, ElevatorAndPivotSubsystem elevatorAndPivotSubsystem, CoralAlgaeDevice coralAlgaeDevice)
    {
        this.driveToReef = driveToReef;
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        this.coralAlgaeDevice = coralAlgaeDevice;
    }

    private Mode selectCommand()
    {
        AutoDriving.DrivePoints drivePoint = driveToReef.getCurrentDriveToPoint();
        AutoDriving.DrivePointModifier drivePointModifier = driveToReef.getCurrentDrivePointModifier();

        switch( drivePoint )
        {
            case BLUE_CORAL_RIGHT:
            case BLUE_CORAL_LEFT:
            case RED_CORAL_RIGHT:
            case RED_CORAL_LEFT:
            {
                return Mode.INTAKE_CORAL;
            }
            case BLUE_PROCESSOR:
            case RED_PROCESSOR:
            {
                return Mode.DEPLOY_ALGAE;
            }
            default:
                // If the robot is at the reef, and the modifier is set to left or right, then we need to deploy a coral
                if( drivePointModifier == AutoDriving.DrivePointModifier.LEFT || drivePointModifier == AutoDriving.DrivePointModifier.RIGHT )
                {
                    return Mode.DEPLOY_CORAL;
                }
                // If the robot is at the reef, and the modifier is set to center, then we need to acquire an algae
                else
                {
                    return Mode.INTAKE_ALGAE;
                }

        }
    }

    public Command getCommand()
    {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry( Mode.INTAKE_CORAL, new PrintCommand( "Intake Coral" ).andThen( new IntakeCoral( coralAlgaeDevice )) ),
                        Map.entry( Mode.DEPLOY_CORAL, new PrintCommand( "Deploy Coral" ).andThen( new DeployCoral( coralAlgaeDevice )) ),
                        Map.entry( Mode.INTAKE_ALGAE, new PrintCommand( "Intake Algae" ).andThen( new IntakeAlgae( coralAlgaeDevice )) ),
                        Map.entry( Mode.DEPLOY_ALGAE, new PrintCommand( "Deploy Algae" ).andThen( new DeployAlgae( coralAlgaeDevice )) )
                ),
                this::selectCommand
        );
    }
}

package frc.robot.commands.elevatorandpivotcommands;

import frc.robot.commands.coralalgae.RunCoralAlgaeDeviceAutomatic;
import frc.robot.commands.swervedrive.AutoDriving;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;

public class SetPivotAngleAutomatically {
    private ElevatorAndPivotSubsystem elevatorAndPivotSubsystem;
    private AutoDriving driveToReef;

    public SetPivotAngleAutomatically( AutoDriving driveToReef, ElevatorAndPivotSubsystem elevatorAndPivotSubsystem )
    {
        this.driveToReef = driveToReef;
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
    }

    public void calculatePivotAngleAndQueue()
    {
        AutoDriving.DrivePoints drivePoint = driveToReef.getCurrentDriveToPoint();
        AutoDriving.DrivePointModifier drivePointModifier = driveToReef.getCurrentDrivePointModifier();
        ElevatorAndPivotSubsystem.ElevatorPositions elevatorPosition = elevatorAndPivotSubsystem.getQueuedElevatorPosition();

        switch( drivePoint )
        {
            case BLUE_CORAL_RIGHT:
            case BLUE_CORAL_LEFT:
            case RED_CORAL_RIGHT:
            case RED_CORAL_LEFT:
            {
                elevatorAndPivotSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.STOWED );
            }
            case BLUE_PROCESSOR:
            case RED_PROCESSOR:
            {
                elevatorAndPivotSubsystem.setQueuedElevatorPosition( ElevatorAndPivotSubsystem.ElevatorPositions.PROCESSOR );
                elevatorAndPivotSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_ALGAE );
            }
            default:
                // If the robot is at the reef, and the modifier is set to left or right, then we need to deploy a coral
                if( drivePointModifier == AutoDriving.DrivePointModifier.LEFT || drivePointModifier == AutoDriving.DrivePointModifier.RIGHT )
                {
                    switch( elevatorPosition )
                    {
                        case L1:
                        case L2:
                        case L3:
                            elevatorAndPivotSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_CORAL );
                        case L4:
                            elevatorAndPivotSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_CORAL_L4 );

                    }
                }
                // If the robot is at the reef, and the modifier is set to center, then we need to acquire an algae
                else
                {
                    elevatorAndPivotSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.INTAKE_ALGAE );
                }

        }
    }
}

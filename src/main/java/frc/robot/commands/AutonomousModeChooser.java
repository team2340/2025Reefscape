package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.coralalgae.RunCoralAlgaeDeviceAutomatic;
import frc.robot.commands.elevatorandpivotcommands.BringElevatorBackDown;
import frc.robot.commands.elevatorandpivotcommands.MovePivotAndElevatorToPosition;
import frc.robot.commands.elevatorandpivotcommands.SetPivotAngleAutomatically;
import frc.robot.commands.swervedrive.AutoDriving;
import frc.robot.subsystems.CoralAlgaeDevice;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Publishes actions onto the dashboard so the drive team can select
 * their autonomous mode right before the match starts
 */
public class AutonomousModeChooser {
    private SwerveSubsystem drivebase = null;
    private AutoDriving driveToReef = null;
    private ElevatorAndPivotSubsystem elevatorAndPivotSubsystem = null;
    private SetPivotAngleAutomatically setPivotAngleAutomatically = null;
    private RunCoralAlgaeDeviceAutomatic runCoralAlgaeDeviceAutomatic;

    private List<SendableChooser<Supplier<Command>>> listOfActions = new ArrayList<>();
    public AutonomousModeChooser( SwerveSubsystem drivebase, AutoDriving driveToReef, ElevatorAndPivotSubsystem elevatorAndPivotSubsystem, SetPivotAngleAutomatically setPivotAngleAutomatically, RunCoralAlgaeDeviceAutomatic runCoralAlgaeDeviceAutomatic ) {
        this.drivebase = drivebase;
        this.driveToReef = driveToReef;
        this.elevatorAndPivotSubsystem = elevatorAndPivotSubsystem;
        this.setPivotAngleAutomatically = setPivotAngleAutomatically;
        this.runCoralAlgaeDeviceAutomatic = runCoralAlgaeDeviceAutomatic;

        for( int i = 0; i < 5; i++ ) {
            SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
            chooser.setDefaultOption( "Do nothing", InstantCommand::new );

            // Add drive positions
            addDriveActions( chooser );

            addScoringActions( chooser );


            SmartDashboard.putData( "AutonomousModeChooser/Auto Action " + (i + 1), chooser );

            listOfActions.add( chooser );
        }
    }

    // These actions move the elevator up to the correct position, score, and then moves the elevator back down to a safe
    // driving position
    private void addScoringActions( SendableChooser<Supplier<Command>> chooser) {
        for( ElevatorAndPivotSubsystem.ElevatorPositions position : ElevatorAndPivotSubsystem.ElevatorPositions.values() )
        {
            chooser.addOption( "Score " + position.name(),
                    () ->  new InstantCommand( () -> {
                        System.out.println("Score " + position.name());
                        elevatorAndPivotSubsystem.setQueuedElevatorPosition( position );
                        setPivotAngleAutomatically.calculatePivotAngleAndQueue();
                    })
                            .andThen( new MovePivotAndElevatorToPosition( elevatorAndPivotSubsystem ) )
                            .andThen( runCoralAlgaeDeviceAutomatic.getCommand() )
                            .andThen( new BringElevatorBackDown( elevatorAndPivotSubsystem ) )
            );
        }

    }

    // These actions will drive the robot up to the right spot
    private void addDriveActions( SendableChooser<Supplier<Command>> chooser) {
        for( AutoDriving.DriveToPoint point : AutoDriving.DriveToPoint.values() )
        {
            if( point.name().contains( "REEF" ) )
            {
                for( AutoDriving.DrivePointModifier modifier : AutoDriving.DrivePointModifier.values()  )
                {
                    Supplier<Command> supplier = () -> new InstantCommand( () -> driveToReef.setDriveToPoint( point )).
                            andThen( new InstantCommand( () -> driveToReef.setDriveToPointModifier( modifier ) ) ).
                            andThen( driveToReef.getAutonomousCommand() );

                    chooser.addOption( "Drive to " + point.name() + " - " + modifier.name(),  supplier);
                }
            }
            else
            {
                Supplier<Command> supplier = () -> new InstantCommand( () -> driveToReef.setDriveToPoint( point )).
                        andThen( driveToReef.getAutonomousCommand() );

                chooser.addOption( "Drive to " + point.name(),  supplier);
            }
        }
    }

    public Command getAutonomousCommand() {
        Command command = new PrintCommand( "Starting Auto" );

        for( SendableChooser<Supplier<Command>> action : listOfActions ) {
            command = command.andThen( action.getSelected().get() );
        }

        return command;
    }




}

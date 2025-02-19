// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonomousModeChooser;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.AutoDriving;
import frc.robot.subsystems.CoralAlgaeDevice;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;
import frc.robot.subsystems.swervedrive.AprilTagPoseProcessing;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final XboxController driverSwerve = new XboxController(0);
  final CommandJoystick streamDeck = new CommandJoystick(1);
  final CommandJoystick streamDeck2 = new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final AutoDriving driveToReef = new AutoDriving( drivebase );
  //private final AprilTagPoseProcessing aprilTagPoseProcessing = new AprilTagPoseProcessing( driveToReef );
  private final AutonomousModeChooser autonomousModeChooser = new AutonomousModeChooser( drivebase, driveToReef);
  private final ElevatorAndPivotSubsystem elevatorSubsystem = new ElevatorAndPivotSubsystem();
  private final CoralAlgaeDevice coralAlgaeDevice = new CoralAlgaeDevice();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> DriverControllerHelpers.getY( driverSwerve ),
                                                                () -> DriverControllerHelpers.getX( driverSwerve) )
                                                            .withControllerRotationAxis( () -> DriverControllerHelpers.getRotation(driverSwerve))
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(() -> !DriverControllerHelpers.isRobotCentric(driverSwerve))
                                                            .robotRelative(() -> DriverControllerHelpers.isRobotCentric(driverSwerve));
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(false);

    elevatorSubsystem.setDefaultCommand( new RepeatCommand( new InstantCommand( elevatorSubsystem::run, elevatorSubsystem )));

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand( "SetReef_1", new InstantCommand( () -> driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.REEF_1 )) );
    NamedCommands.registerCommand( "AlignToReef", driveToReef.getDrivePreciseCommand() );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveWithSetpointGeneratorFieldRelative(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.a().whileTrue( driveToReef.getDrivePreciseCommand().andThen(new PrintCommand("Done!")) );
    driverXbox.x()
            .onTrue( new InstantCommand( elevatorSubsystem::setElevatorToQueuedPosition))
            .onFalse(new InstantCommand( elevatorSubsystem::stopElevator));

    configureStreamDeckBindings();
  }

  private void configureStreamDeckBindings()
  {

    streamDeck.button( 31 )
            .onTrue( new InstantCommand(elevatorSubsystem::startJogElevatorUp, elevatorSubsystem))
            .onFalse(new InstantCommand(elevatorSubsystem::stopJobElevatorUp, elevatorSubsystem));

    streamDeck.button( 32 )
            .onTrue( new InstantCommand(elevatorSubsystem::startJogElevatorDown, elevatorSubsystem))
            .onFalse(new InstantCommand(elevatorSubsystem::stopJobElevatorDown, elevatorSubsystem));

    streamDeck.button( 5 )
            .onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L1)));
    streamDeck.button( 6 )
            .onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L2)));
    streamDeck.button( 10 )
            .onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L3)));
    streamDeck.button( 11 )
            .onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L4)));
    streamDeck.button( 9 )
            .onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE)));

    streamDeck2.button( 1 )
            .onTrue(new InstantCommand( elevatorSubsystem::pivotOut ))
            .onFalse(new InstantCommand( elevatorSubsystem::stopPivot ));


    streamDeck2.button( 2 )
            .onTrue(new InstantCommand( elevatorSubsystem::pivotIn ))
            .onFalse(new InstantCommand( elevatorSubsystem::stopPivot ));

    streamDeck.button(7)
            .onTrue(new InstantCommand( coralAlgaeDevice::runAlgaeIntake))
            .onFalse(new InstantCommand( coralAlgaeDevice::stop));
    streamDeck.button(8)
            .onTrue(new InstantCommand( coralAlgaeDevice::runAlgaeDeploy))
            .onFalse(new InstantCommand( coralAlgaeDevice::stop));

    streamDeck.button(12)
            .onTrue(new InstantCommand( coralAlgaeDevice::runCoralIntake))
            .onFalse(new InstantCommand( coralAlgaeDevice::stop));
    streamDeck.button(13)
            .onTrue(new InstantCommand( coralAlgaeDevice::runCoralDeploy))
            .onFalse(new InstantCommand( coralAlgaeDevice::stop));
    // Debug for buttons
    for( int i = 1; i < 32; i++)
    {
      final int x = i;
      streamDeck.button(i).onTrue( new InstantCommand(() -> {
        System.out.println("Button " + x + " + pressed");
      }));
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    System.out.println("Getting auto command");
    return autonomousModeChooser.getAutonomousCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.coralalgae.*;
import frc.robot.commands.elevatorandpivotcommands.BringElevatorBackDown;
import frc.robot.commands.elevatorandpivotcommands.MovePivotAndElevatorToPosition;
import frc.robot.commands.elevatorandpivotcommands.MovePivotToSafePosition;
import frc.robot.commands.elevatorandpivotcommands.SetPivotAngleAutomatically;
import frc.robot.commands.manualjogcommands.JogElevatorDown;
import frc.robot.commands.manualjogcommands.JogElevatorUp;
import frc.robot.commands.manualjogcommands.JogPivotIn;
import frc.robot.commands.manualjogcommands.JogPivotOut;
import frc.robot.commands.swervedrive.AutoDriving;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralAlgaeDevice;
import frc.robot.subsystems.ElevatorAndPivotSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.swervedrive.AprilTagPoseProcessing;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Map;

import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;

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
  private final ElevatorAndPivotSubsystem elevatorSubsystem = new ElevatorAndPivotSubsystem();
  private final CoralAlgaeDevice coralAlgaeDevice = new CoralAlgaeDevice();
  private final RunCoralAlgaeDeviceAutomatic runCoralAlgaeDeviceAutomatic = new RunCoralAlgaeDeviceAutomatic(driveToReef, elevatorSubsystem, coralAlgaeDevice);
  private final Climber climber = new Climber();

  private final AprilTagPoseProcessing aprilTagPoseProcessing = new AprilTagPoseProcessing( driveToReef );
  private final SetPivotAngleAutomatically setPivotAngleAutomatically = new SetPivotAngleAutomatically( driveToReef, elevatorSubsystem );
  private final AutonomousModeChooser autonomousModeChooser = new AutonomousModeChooser( drivebase, driveToReef, elevatorSubsystem, setPivotAngleAutomatically, runCoralAlgaeDeviceAutomatic, coralAlgaeDevice);
  private final Lights lights = Lights.getInstance();

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
    CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(false);

    elevatorSubsystem.setDefaultCommand( new RepeatCommand( new InstantCommand( elevatorSubsystem::run, elevatorSubsystem )));
    coralAlgaeDevice.setDefaultCommand( new RepeatCommand( new InstantCommand( coralAlgaeDevice::run, coralAlgaeDevice )));

      lights.setDefaultCommand(
              new RepeatCommand(
                      new InstantCommand(lights::run, lights)
              )
      );
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

    driverXbox.a().whileTrue( driveToReef.getCommand().andThen( new RumbleJoystick( driverSwerve ) ) );
    driverXbox.b().onTrue( runCoralAlgaeDeviceAutomatic.getCommand().andThen( new RumbleJoystick( driverSwerve ) )  );
    driverXbox.x().whileTrue( new MovePivotAndElevatorToPosition( elevatorSubsystem ).andThen( new RumbleJoystick( driverSwerve ) )  );
    driverXbox.y().whileTrue( new BringElevatorBackDown( elevatorSubsystem ).andThen( new RumbleJoystick( driverSwerve ) )  );

    driverXbox.povUp().whileTrue( climber.climberOutCommand() );
    driverXbox.povDown().whileTrue( climber.climberInCommand() );

    configureStreamDeckBindings();
  }

  private void configureStreamDeckBindings()
  {
    streamDeck.button( 5 )
            .onTrue( new InstantCommand( () ->
            {
              elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L1);
              elevatorSubsystem.setQueuedPivotAngle(ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_CORAL);
            }));
    streamDeck.button( 6 )
            .onTrue( new InstantCommand( () ->
            {
              elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L2);
              elevatorSubsystem.setQueuedPivotAngle(ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_CORAL);
            }));
    streamDeck.button( 10 )
            .onTrue( new InstantCommand( () ->
            {
              elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L3);
              elevatorSubsystem.setQueuedPivotAngle(ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_CORAL);
            }));
    streamDeck.button( 11 )
            .onTrue( new InstantCommand( () ->
            {
              elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.L4);
              elevatorSubsystem.setQueuedPivotAngle(ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_CORAL);
            }));
    streamDeck.button( 9 )
            .onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE)));

    streamDeck.button( 20 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.REEF_1 )) );
    streamDeck.button( 21 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.REEF_2 )) );
    streamDeck.button( 18 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.REEF_3 )) );
    streamDeck.button( 17 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.REEF_4 )) );
    streamDeck.button( 16 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.REEF_5 )) );
    streamDeck.button( 19 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.REEF_6 )) );
    streamDeck.button( 15 )
            .onTrue( new InstantCommand( () -> {
              driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.PROCESSOR );
              elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE);
              elevatorSubsystem.setQueuedPivotAngle(ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_ALGAE);
            }) );
    streamDeck.button( 22 )
            .onTrue( new InstantCommand( () -> {
              driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.CORAL_STATION_LEFT );
              elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE);
              elevatorSubsystem.setQueuedPivotAngle(ElevatorAndPivotSubsystem.PivotAngles.STOWED);
            }) );
    streamDeck.button( 23 )
            .onTrue( new InstantCommand( () -> {
              driveToReef.setDriveToPoint( AutoDriving.DriveToPoint.CORAL_STATION_RIGHT );
              elevatorSubsystem.setQueuedElevatorPosition(ElevatorAndPivotSubsystem.ElevatorPositions.INTAKE);
              elevatorSubsystem.setQueuedPivotAngle(ElevatorAndPivotSubsystem.PivotAngles.STOWED);
            }) );

    streamDeck.button( 24 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPointModifier( AutoDriving.DrivePointModifier.LEFT )) );
    streamDeck.button( 25 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPointModifier( AutoDriving.DrivePointModifier.CENTER )) );
    streamDeck.button( 26 )
            .onTrue( new InstantCommand( () -> driveToReef.setDriveToPointModifier( AutoDriving.DrivePointModifier.RIGHT )) );

    // Auto drive elevator settings. This will also determine what angle the pivot needs to be
    streamDeck.button( 27 )
            .onTrue( new InstantCommand( () -> {
              elevatorSubsystem.setQueuedElevatorPosition( ElevatorAndPivotSubsystem.ElevatorPositions.L1 );
              setPivotAngleAutomatically.calculatePivotAngleAndQueue();
            }) );
    streamDeck.button( 28 )
            .onTrue( new InstantCommand( () -> {
              elevatorSubsystem.setQueuedElevatorPosition( ElevatorAndPivotSubsystem.ElevatorPositions.L2 );
              setPivotAngleAutomatically.calculatePivotAngleAndQueue();
            }) );
    streamDeck.button( 29 )
            .onTrue( new InstantCommand( () -> {
              elevatorSubsystem.setQueuedElevatorPosition( ElevatorAndPivotSubsystem.ElevatorPositions.L3 );
              setPivotAngleAutomatically.calculatePivotAngleAndQueue();
            }) );
    streamDeck.button( 30 )
            .onTrue( new InstantCommand( () -> {
              elevatorSubsystem.setQueuedElevatorPosition( ElevatorAndPivotSubsystem.ElevatorPositions.L4 );
              setPivotAngleAutomatically.calculatePivotAngleAndQueue();
            }) );

    // Manual Jog Menu Actions
    streamDeck2.button( 1 ).whileTrue( new JogPivotOut( elevatorSubsystem ) );
    streamDeck2.button( 2 ).whileTrue( new JogPivotIn( elevatorSubsystem ) );
    streamDeck.button( 31 ).whileTrue( new JogElevatorUp( elevatorSubsystem ) );
    streamDeck.button( 32 ).whileTrue( new JogElevatorDown( elevatorSubsystem ) );

    streamDeck.button(7).toggleOnTrue(new IntakeAlgae(coralAlgaeDevice));

    streamDeck.button(8).onTrue( new DeployAlgae( coralAlgaeDevice) );

    streamDeck.button(12).onTrue( new IntakeCoral( coralAlgaeDevice ) );

    streamDeck.button(13).onTrue( new DeployCoral( coralAlgaeDevice  ) );

    streamDeck.button(1).onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.STOWED )));
    streamDeck.button(2).onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.INTAKE_ALGAE )));
    streamDeck.button(3).onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_CORAL )));
    streamDeck.button(4).onTrue( new InstantCommand( () -> elevatorSubsystem.setQueuedPivotAngle( ElevatorAndPivotSubsystem.PivotAngles.DEPLOY_ALGAE )));


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
    return autonomousModeChooser.getAutonomousCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void resetPIDControllers()
  {
    elevatorSubsystem.resetPIDControllers();
  }

  public void publishReadyChecks()
  {
      elevatorSubsystem.publishReadyChecks();
      coralAlgaeDevice.publishReadyChecks();
      SmartDashboard.putBoolean("ReadyChecks/BackCamera", Vision.Cameras.BACK_CAM.camera.isConnected());
      SmartDashboard.putBoolean("ReadyChecks/LeftCamera", Vision.Cameras.LEFT_CAM.camera.isConnected());
      SmartDashboard.putBoolean("ReadyChecks/RightCamera", Vision.Cameras.RIGHT_CAM.camera.isConnected());

      Map<String, SwerveModule> swerveModules = drivebase.getSwerveDrive().getModuleMap();
      for(Map.Entry<String, SwerveModule> entry : swerveModules.entrySet() )
      {
          SmartDashboard.putBoolean("ReadyChecks/Swerve_" + entry.getKey() + "Encoder", !entry.getValue().getAbsoluteEncoderReadIssue() );
      }
  }
}

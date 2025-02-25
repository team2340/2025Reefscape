package frc.robot.commands.swervedrive;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.constants.AutoDrivingConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class AutoDriving {
    private final SendableChooser<DriveToPoint> driveToPointDashboardChooser = new SendableChooser<>();
    private final SendableChooser<DrivePointModifier> driveToPointModifierDashboardChooser = new SendableChooser<>();
    private final Vision vision;
    public enum DrivePointModifier {
        LEFT, CENTER, RIGHT
    }

    private Command driveToPoseCommand = null;

    public enum DriveToPoint {
        REEF_1(DrivePoints.RED_REEF_1, DrivePoints.BLUE_REEF_1),
        REEF_2(DrivePoints.RED_REEF_2, DrivePoints.BLUE_REEF_2),
        REEF_3(DrivePoints.RED_REEF_3, DrivePoints.BLUE_REEF_3),
        REEF_4(DrivePoints.RED_REEF_4, DrivePoints.BLUE_REEF_4),
        REEF_5(DrivePoints.RED_REEF_5, DrivePoints.BLUE_REEF_5),
        REEF_6(DrivePoints.RED_REEF_6, DrivePoints.BLUE_REEF_6),

        CORAL_STATION_LEFT(DrivePoints.RED_CORAL_LEFT, DrivePoints.BLUE_CORAL_LEFT),
        CORAL_STATION_RIGHT(DrivePoints.RED_CORAL_RIGHT, DrivePoints.BLUE_CORAL_RIGHT),
        PROCESSOR(DrivePoints.RED_PROCESSOR, DrivePoints.BLUE_PROCESSOR),
        ;

        private DrivePoints redPoint;
        private DrivePoints bluePoint;
        DriveToPoint( DrivePoints redPoint, DrivePoints bluePoint)
        {
            this.redPoint = redPoint;
            this.bluePoint = bluePoint;
        }
    }
    public enum DrivePoints{
        RED_REEF_1(
                //new Pose2d( 14.82, 4.01, Rotation2d.fromDegrees( 180.00 ) ),
                new Pose2d( 15.65, 4.01, Rotation2d.fromDegrees( 180.00 ) ),
                7
        ),
        RED_REEF_2(
                new Pose2d( 14.01, 5.53, Rotation2d.fromDegrees( -122.81 ) ),
                8
        ),
        RED_REEF_3(
                new Pose2d( 12.16, 5.60, Rotation2d.fromDegrees( -60.00 ) ),
                9
        ),
        RED_REEF_4(
                new Pose2d( 11.28, 4.04, Rotation2d.fromDegrees( 0.00 ) ),
                10
        ),
        RED_REEF_5(
                new Pose2d( 12.14, 2.50, Rotation2d.fromDegrees( 58.26 ) ),
                11
        ),
        RED_REEF_6(
                new Pose2d( 13.91, 2.52, Rotation2d.fromDegrees( 120.00 ) ),
                7
        ),

        BLUE_CORAL_LEFT(
                new Pose2d(1.14, 6.89, Rotation2d.fromDegrees( 305 )),
                13),

        BLUE_CORAL_RIGHT(
                new Pose2d(1.32, 1.09, Rotation2d.fromDegrees( 56 )),
                12),

        BLUE_PROCESSOR(
                new Pose2d(11.29, 7.51, Rotation2d.fromDegrees( 90 )),
                3),

        BLUE_REEF_1(
                new Pose2d( 2.79, 4.01, Rotation2d.fromDegrees( 0.00 ) ),
                18
        ),
        BLUE_REEF_2(
                new Pose2d( 3.60, 2.50, Rotation2d.fromDegrees( 60 ) ),
                17
        ),
        BLUE_REEF_3(
                new Pose2d( 5.30, 2.57, Rotation2d.fromDegrees( 124.42 ) ),
                22
        ),
        BLUE_REEF_4(
                new Pose2d( 6.28, 4.01, Rotation2d.fromDegrees( -180.00 ) ),
                21
        ),
        BLUE_REEF_5(
                new Pose2d( 5.43, 5.44, Rotation2d.fromDegrees( -120 ) ),
                20
        ),
        BLUE_REEF_6(
                new Pose2d( 3.59, 5.54, Rotation2d.fromDegrees( -60 ) ),
                19
        ),

        RED_CORAL_LEFT(
                new Pose2d(16.18,1.06, Rotation2d.fromDegrees( -52 )),
                1),

        RED_CORAL_RIGHT(
                new Pose2d(16.36, 6.90, Rotation2d.fromDegrees( 52 )),
                2),
        
        RED_PROCESSOR(
                new Pose2d(5.98, 0.79, Rotation2d.fromDegrees( -90 )),
                16);


        private final Pose2d pose;
        private final Integer aprilTagId;
        DrivePoints( Pose2d centerPose, Integer aprilTagId )
        {
            this.pose = centerPose;
            this.aprilTagId = aprilTagId;
        }

        public Pose2d getPoint( DrivePointModifier modifier )
        {
            return pose;
        }

        public int getAprilTagId()
        {
            return aprilTagId;
        }

        public boolean hasAprilTag()
        {
            return aprilTagId != null;
        }

    }

    private DrivePoints currentDrivePoint = DrivePoints.BLUE_REEF_1;
    private DrivePointModifier currentDrivePointModifier = DrivePointModifier.CENTER;

    private final SwerveSubsystem swerve;

    private final ProfiledPIDController xController = new ProfiledPIDController(
            AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_Kp,
            AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_Ki,
            AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_Kd,
            new TrapezoidProfile.Constraints(
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_MAX_SPEED,
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_MAX_ACCEL ) );

    private final ProfiledPIDController yController =
            new ProfiledPIDController(
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_Kp,
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_Ki,
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_Kd,
                    new TrapezoidProfile.Constraints(
                            AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_MAX_SPEED,
                            AutoDrivingConstants.PRECISE_AUTO_DRIVE_TRANSLATION_PID_MAX_ACCEL ) );

    private final ProfiledPIDController rotationController =
            new ProfiledPIDController(
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_ROTATION_PID_Kp,
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_ROTATION_PID_Ki,
                    AutoDrivingConstants.PRECISE_AUTO_DRIVE_ROTATION_PID_Kd,
                    new TrapezoidProfile.Constraints(
                            AutoDrivingConstants.PRECISE_AUTO_DRIVE_ROTATION_PID_MAX_SPEED,
                            AutoDrivingConstants.PRECISE_AUTO_DRIVE_ROTATION_PID_MAX_ACCEL ) );

    private boolean isAtTarget = false;
    private PhotonTrackedTarget visionLastTarget = null;
    private Vision.Cameras visionLastCamera = null;
    private Pose2d visionGoalPose = null;
    private int preciseDriveCount = 0;
    Field2d field = new Field2d();           


    public AutoDriving( SwerveSubsystem swerve )
    {
        this.swerve = swerve;
        this.vision = swerve.getVision();
        for( DriveToPoint point : DriveToPoint.values() ) {
            driveToPointDashboardChooser.addOption( point.name(), point );
        }
        driveToPointDashboardChooser.setDefaultOption( DriveToPoint.REEF_1.name(),  DriveToPoint.REEF_1 );
        driveToPointDashboardChooser.onChange( this::setDriveToPoint );

        driveToPointModifierDashboardChooser.addOption( DrivePointModifier.LEFT.name(), DrivePointModifier.LEFT );
        driveToPointModifierDashboardChooser.setDefaultOption( DrivePointModifier.CENTER.name(), DrivePointModifier.CENTER );
        driveToPointModifierDashboardChooser.addOption( DrivePointModifier.RIGHT.name(), DrivePointModifier.RIGHT );
        driveToPointModifierDashboardChooser.onChange( this::setDriveToPointModifier );

        setDriveToPoint( DriveToPoint.REEF_1 );

        rotationController.enableContinuousInput( 0, 360 );

        xController.setTolerance(AutoDrivingConstants.PRECISE_POSITION_THRESHOLD, 0.03);
        yController.setTolerance(AutoDrivingConstants.PRECISE_POSITION_THRESHOLD, 0.03);
        rotationController.setTolerance(3);
        SmartDashboard.putData("AutoDriving/Drive Point", driveToPointDashboardChooser );
        SmartDashboard.putData("AutoDriving/Drive Point Modifier", driveToPointModifierDashboardChooser );
        SmartDashboard.putData("AutoDriving/VisionGoalPose", field);

        buildDriveToPoseCommand();
    }

    public void setDriveToPoint(DriveToPoint driveToPoint) {

        var alliance = DriverStation.getAlliance();
        boolean isRedAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;

        DrivePoints point = isRedAlliance ? driveToPoint.redPoint : driveToPoint.bluePoint;

        SmartDashboard.putString( "AutoDriving/Drive To Point Name", point.name() );

        driveToPointDashboardChooser.setDefaultOption( point.name(), driveToPoint );
        currentDrivePoint = point;

        field.setRobotPose(currentDrivePoint.pose);
    }

    public void setDriveToPointModifier(DrivePointModifier pointModifier) {
        SmartDashboard.putString( "AutoDriving/Drive To Point Modifier Name", pointModifier.name() );

        driveToPointModifierDashboardChooser.setDefaultOption( pointModifier.name(), pointModifier );
        currentDrivePointModifier = pointModifier;
    }
    
    private Transform2d getTagToPoseFromPointModifier()
    {
        switch( currentDrivePointModifier ){
                case CENTER:
                        return AutoDrivingConstants.TAG_TO_GOAL_CENTER;
                case LEFT:
                        return AutoDrivingConstants.TAG_TO_GOAL_LEFT;
                case RIGHT:
                        return AutoDrivingConstants.TAG_TO_GOAL_RIGHT;
        }

        return null;
    }

    public Command getCommand()
    {
        return getDriveToPoseCommand()
                .until( () -> {
                   Pose2d currentPose = swerve.getPose();
                   Pose2d targetPose = currentDrivePoint.getPoint( currentDrivePointModifier );
                   double distance = currentPose.getTranslation().getDistance( targetPose.getTranslation() );
                   return distance < AutoDrivingConstants.PATHPLANNER_TO_PRECISE_DRIVE_DISTANCE_THRESHOLD;
                } )
                .andThen( getDrivePreciseCommand() );
                
    }

    private void buildDriveToPoseCommand()
    {
        Map<String, Command> entryMap = new HashMap<>();
        for( DrivePoints point : DrivePoints.values() )
        {
            entryMap.put( point.name() + DrivePointModifier.LEFT.name(), swerve.driveToPose( point.getPoint( DrivePointModifier.LEFT ) ) );
            entryMap.put( point.name() + DrivePointModifier.CENTER.name(), swerve.driveToPose( point.getPoint( DrivePointModifier.CENTER ) ) );
            entryMap.put( point.name() + DrivePointModifier.RIGHT.name(), swerve.driveToPose( point.getPoint( DrivePointModifier.RIGHT ) ) );
        }

        driveToPoseCommand = new SelectCommand<>( entryMap, () -> currentDrivePoint.name() + currentDrivePointModifier.name() );
    }
    /**
     * Drive to the desired location.
     * This uses PathPlanner and gets the robot roughly to the correct area
     */
    private Command getDriveToPoseCommand()
    {
        return driveToPoseCommand;
    }

    public Command getAutonomousCommand()
    {
        Map<String, Command> entryMap = new HashMap<>();
        for( DrivePoints point : DrivePoints.values() )
        {
            entryMap.put( point.name() + DrivePointModifier.LEFT.name(), swerve.driveToPose( point.getPoint( DrivePointModifier.LEFT ) ) );
            entryMap.put( point.name() + DrivePointModifier.CENTER.name(), swerve.driveToPose( point.getPoint( DrivePointModifier.CENTER ) ) );
            entryMap.put( point.name() + DrivePointModifier.RIGHT.name(), swerve.driveToPose( point.getPoint( DrivePointModifier.RIGHT ) ) );
        }

        return new SelectCommand<>( entryMap, () -> currentDrivePoint.name() + currentDrivePointModifier.name() ).until( () -> {
                    Pose2d currentPose = swerve.getPose();
                    Pose2d targetPose = currentDrivePoint.getPoint( currentDrivePointModifier );
                    double distance = currentPose.getTranslation().getDistance( targetPose.getTranslation() );
                    return distance < AutoDrivingConstants.PATHPLANNER_TO_PRECISE_DRIVE_DISTANCE_THRESHOLD;
                } )
                .andThen( getDrivePreciseCommand() );
    }

    /**
     * Drives to the desired location using PID Controllers
     * This should be used once the robot is close to the location, but
     * not quite there yet. This will get the robot to the exact position
     */
    public Command getDrivePreciseCommand()
    {
        return Commands.runOnce( () -> {
            System.out.println("Starting drive precise");
            // Sets the PID controller's current robot position initially
            Pose2d robotPose = swerve.getPose();
            xController.reset( robotPose.getX() );
            yController.reset( robotPose.getY() );
            rotationController.reset( robotPose.getRotation().getDegrees() );
            isAtTarget = false;
            visionLastTarget = null;
            preciseDriveCount = 0;
            visionGoalPose = null;
        } ).andThen( Commands.run( () -> {
            // Uses the PID controller to adjust the X, Y, and rotation position of the robot
            Pose2d robotPose = swerve.getPose();
            preciseDriveCount++;

            setPoseToAprilTag();

            if( !currentDrivePoint.hasAprilTag() || visionGoalPose == null) // If there is no AprilTag associated with the location, then just try to use odometry to get close to where we need to be
            {
                xController.setGoal( currentDrivePoint.getPoint( currentDrivePointModifier ).getX() );
                yController.setGoal( currentDrivePoint.getPoint( currentDrivePointModifier ).getY() );
                rotationController.setGoal( currentDrivePoint.getPoint( currentDrivePointModifier ).getRotation().getDegrees() );
            }

            double xDriveSpeed = xController.calculate( robotPose.getX() );
            double yDriveSpeed = yController.calculate( robotPose.getY() );
            double rotationSpeed = rotationController.calculate( robotPose.getRotation().getDegrees() );

            if( xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint() && preciseDriveCount > 10 )
            {
                isAtTarget = true;
            }

            // Send the calculated drive values to the swerve drive subsystem
            swerve.driveFieldOriented( new ChassisSpeeds( xDriveSpeed, yDriveSpeed, rotationSpeed ) );

        }, swerve) )
        .until( () -> isAtTarget );
    }

    public void setPoseToAprilTag()
    {
        if( currentDrivePoint.hasAprilTag() ) // If there is an AprilTag associated with the location, then position the robot in relative to that
            {
                Pose2d robotPose = swerve.getPose();
                // Code from https://github.com/STMARobotics/frc-7028-2023/blob/301928718c80cc6db3c29a6bfad8b36483baf1b4/src/main/java/frc/robot/commands/ChaseTagCommand.java
                double bestAmbiguity = Double.MAX_VALUE;
                for(Vision.Cameras camera : Vision.Cameras.values() )
                {
                    Optional<PhotonPipelineResult> visionResults = camera.getLatestResult();
                    if ( visionResults.isPresent() )
                    {
                        PhotonPipelineResult result = visionResults.get();
                        Optional<PhotonTrackedTarget> targetOpt = result.getTargets().stream()
                                .filter( t -> t.getFiducialId() == currentDrivePoint.getAprilTagId() )
                                .findFirst();

                        SmartDashboard.putBoolean("AutoDriving/AprilTagFound", targetOpt.isPresent());
                        if( targetOpt.isPresent() )
                        {
                            PhotonTrackedTarget target = targetOpt.get();
                            if( target.getPoseAmbiguity() >= Vision.maximumAmbiguity || target.getPoseAmbiguity() >= bestAmbiguity ) {
                                continue;
                            }
                            bestAmbiguity = target.getPoseAmbiguity();
                            if( !target.equals( visionLastTarget ) )
                            {
                                visionLastTarget = target;

                                var camToTarget = target.getBestCameraToTarget();
                                var transform = new Transform2d(
                                        camToTarget.getTranslation().toTranslation2d(),
                                        camToTarget.getRotation().toRotation2d()
                                );

                                Transform2d CAMERA_TO_ROBOT = new Transform2d(
                                        new Translation2d( camera.robotToCamTransform.getX(), camera.robotToCamTransform.getY() ),
                                        new Rotation2d( camera.robotToCamTransform.getRotation().getZ() ) );

                                var cameraPose = robotPose.transformBy( CAMERA_TO_ROBOT );
                                Pose2d targetPose = cameraPose.transformBy( transform );

                                // Go either left/center/right of the april tag depending on what the driver has chosen
                                visionGoalPose = targetPose.transformBy( getTagToPoseFromPointModifier() );

                                Transform2d tagToRobot = targetPose.minus(cameraPose);

                                if( tagToRobot != null )
                                {
                                    SmartDashboard.putNumber("AutoDriving/AprilTagId", target.getFiducialId());
                                    SmartDashboard.putString("AutoDriving/AprilTag-To-Robot-Pose", "X: " + (double) Math.round( 100 * tagToRobot.getX() ) / 100 + ", Y: " + (double) Math.round( 100 * tagToRobot.getY() ) / 100 + ", Rotation: " + Math.round(tagToRobot.getRotation().getDegrees()) );
                                }

                                if( visionGoalPose != null ) {
                                    field.setRobotPose(visionGoalPose);
                                    if( camera == visionLastCamera )
                                    {
                                        visionLastCamera = camera;
                                        xController.setGoal(visionGoalPose.getX());
                                        yController.setGoal(visionGoalPose.getY());
                                        rotationController.setGoal(visionGoalPose.getRotation().getDegrees());
                                        return;
                                    }
                                    visionLastCamera = camera;
                                }
                            }
                        }
                    }
                }

                if( visionGoalPose != null)
                {
                    xController.setGoal(visionGoalPose.getX());
                    yController.setGoal(visionGoalPose.getY());
                    rotationController.setGoal(visionGoalPose.getRotation().getDegrees());
                }
            }
    }

    public DrivePoints getCurrentDriveToPoint()
    {
        return currentDrivePoint;
    }

    public DrivePointModifier getCurrentDrivePointModifier()
    {
        return currentDrivePointModifier;
    }
}

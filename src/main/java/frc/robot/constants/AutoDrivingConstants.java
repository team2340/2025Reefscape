package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoDrivingConstants {

    // The distance (meters) that the code will switch from path planning to PID control to get to the desired pose
    public static final double PATHPLANNER_TO_PRECISE_DRIVE_DISTANCE_THRESHOLD = 0.2;

    // The distance that the code will consider the robot at the precise location. This will cause the command to stop and then any other commands to run
    public static final double PRECISE_POSITION_THRESHOLD = 0.07;

    public static final Transform2d TAG_TO_GOAL_CENTER = new Transform2d(new Translation2d(.24, 0), Rotation2d.fromDegrees(179.0));
    public static final Transform2d TAG_TO_GOAL_LEFT = new Transform2d(new Translation2d(.24, -0.24), Rotation2d.fromDegrees(179.0));
    public static final Transform2d TAG_TO_GOAL_RIGHT = new Transform2d(new Translation2d(.24, 0.23), Rotation2d.fromDegrees(-179.0));

    public static final Transform2d CORAL_STATION_TAG_TO_GOAL = new Transform2d( new Translation2d(.42, 0), Rotation2d.fromDegrees(2));
    public static final Transform2d PROCESSOR_TAG_TO_GOAL = new Transform2d( new Translation2d(1, 0), Rotation2d.fromDegrees(180));;


    // PID Controller values for the precise drive to target
    public static final double PRECISE_AUTO_DRIVE_TRANSLATION_PID_MAX_SPEED = Constants.MAX_SPEED * 0.8;
    public static final double PRECISE_AUTO_DRIVE_TRANSLATION_PID_MAX_ACCEL = 4;

    public static final double PRECISE_AUTO_DRIVE_TRANSLATION_PID_Kp = 2.0;
    public static final double PRECISE_AUTO_DRIVE_TRANSLATION_PID_Ki = 0.1;
    public static final double PRECISE_AUTO_DRIVE_TRANSLATION_PID_Kd = 0.008;

    public static final double PRECISE_AUTO_DRIVE_ROTATION_PID_MAX_SPEED = 100;
    public static final double PRECISE_AUTO_DRIVE_ROTATION_PID_MAX_ACCEL = 100;

    public static final double PRECISE_AUTO_DRIVE_ROTATION_PID_Kp = 0.20;
    public static final double PRECISE_AUTO_DRIVE_ROTATION_PID_Ki = 0.05;
    public static final double PRECISE_AUTO_DRIVE_ROTATION_PID_Kd = .0;


}

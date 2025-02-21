package frc.robot.constants;

public class ElevatorAndPivotConstants {
    public final static int ELEVATOR_CAN_ID = 9;
    public final static int ELEVATOR_FOLLOWER_CAN_ID = 10;
    public final static int PIVOT_CAN_ID = 11;

    public static final double MINIMUM_PIVOT_ANGLE_FOR_ELEVATOR_MOVEMENT = 5.0;

    public static final double ELEVATOR_MAX_SPEED = 100;
    public static final double ELEVATOR_MAX_ACCEL = 200;
    public static final double ELEVATOR_kP = .9;
    public static final double ELEVATOR_kI = 0;
    public static final double ELEVATOR_kD = 0.02;

    public static final double ELEVATOR_kS = 0.5;
    public static final double ELEVATOR_kG = 0;
    public static final double ELEVATOR_kV = 0;
    public static final double ELEVATOR_kA = 0;

    public static final double PIVOT_MAX_SPEED = 10;
    public static final double PIVOT_MAX_ACCEL = 10;
    public static final double PIVOT_kP = 0;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;
}

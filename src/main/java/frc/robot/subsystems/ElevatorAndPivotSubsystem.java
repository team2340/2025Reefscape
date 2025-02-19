package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorAndPivotConstants;

public class ElevatorAndPivotSubsystem extends SubsystemBase {
    private DigitalInput bottomLimitSwitch = new DigitalInput(0);
    private ElevatorPositions queuedElevatorPosition = null;
    private PivotAngles queuedPivotAngle = null;

    private ElevatorPositions desiredElevatorPosition = null;
    private PivotAngles desiredPivotAngle = null;

    private static int elevatorCAN = 9;
    private static int elevatorFollowerCAN = 10;
    private SparkMax elevatorSparkMax = new SparkMax( elevatorCAN, SparkLowLevel.MotorType.kBrushless );
    private SparkMax elevatorFollowerSparkMax = new SparkMax( elevatorFollowerCAN, SparkLowLevel.MotorType.kBrushless );
    private SparkMax pivotSparkMax = new SparkMax( 11, SparkLowLevel.MotorType.kBrushless);

    public void pivotOut() {
        pivotSparkMax.set(-0.3);
    }

    public void pivotIn() {
        pivotSparkMax.set(0.3);
    }
    public void stopPivot() {
        pivotSparkMax.set(0);
    }

    public enum ElevatorPositions {
        INTAKE(10, 10),
        L1(20, 10),
        L2(62, 10),
        L3(135, 10),
        L4(225, 10);

        private long encoderPosition;
        private long minimumPivotEncoder;
        ElevatorPositions(long encoderPosition, int minimumPivotEncoder) {
            this.encoderPosition = encoderPosition;
            this.minimumPivotEncoder = minimumPivotEncoder;
        }

        public long getEncoderPosition() {
            return encoderPosition;
        }

        public long getMinimumPivotEncoder() {
            return minimumPivotEncoder;
        }
    }

    public enum PivotAngles {
        STOWED( 0 ),
        DEPLOY_CORAL(5),
        DEPLOY_CORAL_L4(10),
        DEPLOY_ALGAE(90);

        private long encoderPosition;
        PivotAngles(long encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public long getEncoderPosition() {
            return encoderPosition;
        }
    }


    ProfiledPIDController elevatorPIDController = new ProfiledPIDController(
            ElevatorAndPivotConstants.ELEVATOR_kP,
            ElevatorAndPivotConstants.ELEVATOR_kI,
            ElevatorAndPivotConstants.ELEVATOR_kD,
            new TrapezoidProfile.Constraints( ElevatorAndPivotConstants.ELEVATOR_MAX_SPEED, ElevatorAndPivotConstants.ELEVATOR_MAX_ACCEL )
    );

    ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
            ElevatorAndPivotConstants.ELEVATOR_kS,
            ElevatorAndPivotConstants.ELEVATOR_kG,
            ElevatorAndPivotConstants.ELEVATOR_kV,
            ElevatorAndPivotConstants.ELEVATOR_kA
    );

    ProfiledPIDController pivotPIDController = new ProfiledPIDController(
            ElevatorAndPivotConstants.PIVOT_kP,
            ElevatorAndPivotConstants.PIVOT_kI,
            ElevatorAndPivotConstants.PIVOT_kP,
            new TrapezoidProfile.Constraints( ElevatorAndPivotConstants.PIVOT_MAX_SPEED, ElevatorAndPivotConstants.PIVOT_MAX_ACCEL )
    );

    public ElevatorAndPivotSubsystem()
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(elevatorCAN, true);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        elevatorFollowerSparkMax.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.idleMode(SparkBaseConfig.IdleMode.kBrake);
        elevatorSparkMax.configure(config2, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


    }
    private double getCurrentElevatorEncoderPosition() {
        return elevatorSparkMax.getEncoder().getPosition();
    }

    private long getCurrentPivotEncoderPosition() {
        return 0;
    }

    private void movePivotToDesiredAngle(double position) {

    }

    private void moveElevatorToDesiredPosition( long position ) {

        double pidControllerValue = MathUtil.clamp(elevatorPIDController.calculate( getCurrentElevatorEncoderPosition()), -12, 12);
        elevatorSparkMax.setVoltage( pidControllerValue + elevatorFeedforward.calculate( elevatorSparkMax.getEncoder().getVelocity()));
    }

    public void startJogElevatorUp()
    {
        desiredElevatorPosition = null;
        elevatorSparkMax.set( 0.2 );
    }

    public void stopJobElevatorUp()
    {
        elevatorSparkMax.set( 0.0 );

    }

    public void startJogElevatorDown()
    {
        desiredElevatorPosition = null;
        elevatorSparkMax.set( -0.2 );

    }

    public void stopJobElevatorDown()
    {
        elevatorSparkMax.set( -0.0 );
    }

    public void setQueuedElevatorPosition(ElevatorPositions elevatorPosition)
    {
        queuedElevatorPosition = elevatorPosition;
    }
    public void setElevatorToQueuedPosition()
    {
        desiredElevatorPosition = queuedElevatorPosition;
        desiredPivotAngle = queuedPivotAngle;
        elevatorPIDController.setGoal( desiredElevatorPosition.getEncoderPosition());
        elevatorPIDController.reset( getCurrentElevatorEncoderPosition() );
    }

    public void stopElevator()
    {
        elevatorSparkMax.set(0);
    }

    public void run() {

        // If the elevator is not where it wants to be, check the pivot angle to make sure it is safe before moving.
        // If the pivot angle is not safe before moving, then move the pivot motor first before moving the elevator.
        // Once the pivot is finished moving to the safe position, then move the elevator
        SmartDashboard.putNumber("Current Elevator Encoder", getCurrentElevatorEncoderPosition());
        if( !bottomLimitSwitch.get() )
        {
            elevatorSparkMax.getEncoder().setPosition(0);
            //elevatorSparkMax.set(0);
        }

        if( desiredElevatorPosition != null )
        {
            if( getCurrentElevatorEncoderPosition() != desiredElevatorPosition.getEncoderPosition() ) {
                if( getCurrentPivotEncoderPosition() < ElevatorAndPivotConstants.MINIMUM_PIVOT_ANGLE_FOR_ELEVATOR_MOVEMENT ) {
                   // movePivotToDesiredAngle( ElevatorAndPivotConstants.MINIMUM_PIVOT_ANGLE_FOR_ELEVATOR_MOVEMENT );
                   // return;
                }
                moveElevatorToDesiredPosition( desiredElevatorPosition.getEncoderPosition() );
            } else {

                // Move the pivot motor if it's position is not currently in the right position, and the current desired
                // pivot angle is not smaller than the position's minimum pivot angle
                if( desiredPivotAngle.getEncoderPosition() >= desiredElevatorPosition.getMinimumPivotEncoder() ) {
                    movePivotToDesiredAngle( desiredPivotAngle.getEncoderPosition() );
                }
            }
        }


    }

}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorAndPivotConstants;

public class ElevatorAndPivotSubsystem extends SubsystemBase {
    private boolean homed = false;
    private DigitalInput bottomLimitSwitch = new DigitalInput(0);
    private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);

    private ElevatorPositions queuedElevatorPosition = ElevatorPositions.L1;
    private ElevatorPositions desiredElevatorPosition = ElevatorPositions.INTAKE;

    private PivotAngles queuedPivotAngle = PivotAngles.STOWED;
    private PivotAngles desiredPivotAngle = PivotAngles.STOWED;

    private SparkMax elevatorSparkMax = new SparkMax( ElevatorAndPivotConstants.ELEVATOR_CAN_ID, SparkLowLevel.MotorType.kBrushless );
    private SparkMax elevatorFollowerSparkMax = new SparkMax( ElevatorAndPivotConstants.ELEVATOR_FOLLOWER_CAN_ID, SparkLowLevel.MotorType.kBrushless );
    private SparkMax pivotSparkMax = new SparkMax( ElevatorAndPivotConstants.PIVOT_CAN_ID, SparkLowLevel.MotorType.kBrushless);

    public void pivotOut() {
        pivotSparkMax.set(-0.3);
    }
    public void pivotIn() {
        pivotSparkMax.set(0.3);
    }
    public void stopPivot() {
        pivotSparkMax.set(0);
    }

    public void resetPIDControllers() {
        pivotPIDController.reset(getCurrentPivotEncoderPosition());
        elevatorPIDController.reset(getCurrentElevatorEncoderPosition());
    }

    public enum ElevatorPositions {
        INTAKE(0, 10),
        L1(20 +5, 10),
        L2(46, 10),
        L3(120, 10),
        L4(219, 10),
        PROCESSOR(0, 10);

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
        STOWED( 0.4866 ),
        DEPLOY_CORAL(.530),
        INTAKE_ALGAE(.630),
        DEPLOY_CORAL_L4(.535),
        DEPLOY_ALGAE(.728);

        private double encoderPosition;
        PivotAngles(double encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public double getEncoderPosition() {
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
            ElevatorAndPivotConstants.PIVOT_kD,
            new TrapezoidProfile.Constraints( ElevatorAndPivotConstants.PIVOT_MAX_SPEED, ElevatorAndPivotConstants.PIVOT_MAX_ACCEL )
    );

    public ElevatorAndPivotSubsystem()
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(ElevatorAndPivotConstants.ELEVATOR_CAN_ID, true);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        elevatorFollowerSparkMax.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.idleMode(SparkBaseConfig.IdleMode.kBrake);
        elevatorSparkMax.configure(config2, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        SparkMaxConfig config3 = new SparkMaxConfig();
        config3.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotSparkMax.configure(config3, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


        elevatorPIDController.setGoal( 0 );
        elevatorPIDController.reset(getCurrentElevatorEncoderPosition());

        pivotPIDController.setGoal(PivotAngles.STOWED.getEncoderPosition());
        pivotPIDController.reset(getCurrentPivotEncoderPosition());

        pivotPIDController.setTolerance(0.005);

    }
    public double getCurrentElevatorEncoderPosition() {
        return elevatorSparkMax.getEncoder().getPosition();
    }

    public double getCurrentPivotEncoderPosition() {
        return pivotEncoder.get();
    }

    private void movePivotToDesiredAngle() {
        double pidControllerValue = MathUtil.clamp(pivotPIDController.calculate( getCurrentPivotEncoderPosition() ), -12, 12);
        pivotSparkMax.setVoltage( -pidControllerValue );
    }

    private void moveElevatorToDesiredPosition() {
        double pidControllerValue = MathUtil.clamp(elevatorPIDController.calculate( getCurrentElevatorEncoderPosition()), -12, 12);
        elevatorSparkMax.setVoltage( pidControllerValue + elevatorFeedforward.calculate( elevatorSparkMax.getEncoder().getVelocity()));
    }

    public void startJogElevatorUp()
    {
        desiredElevatorPosition = null;
        elevatorSparkMax.set( 0.2 );
    }

    public void startJogElevatorDown()
    {
        desiredElevatorPosition = null;
        elevatorSparkMax.set( -0.2 );
    }

    public void setQueuedPivotAngle(PivotAngles angle )
    {
        queuedPivotAngle = angle;
    }

    public void setPivotToQueuedPosition()
    {
        desiredPivotAngle = queuedPivotAngle;
        setPivotPIDControllerSetpoint( desiredPivotAngle.getEncoderPosition() );
    }
    public void setQueuedElevatorPosition(ElevatorPositions elevatorPosition)
    {
        queuedElevatorPosition = elevatorPosition;
    }

    public ElevatorPositions getCurrentDesiredElevatorPosition()
    {
        return desiredElevatorPosition;
    }

    public ElevatorPositions getQueuedElevatorPosition()
    {
        return queuedElevatorPosition;
    }

    public void setElevatorToQueuedPosition()
    {
        desiredElevatorPosition = queuedElevatorPosition;
        setElevatorPIDControllerSetpoint( desiredElevatorPosition.getEncoderPosition() );
    }

    public void setElevatorPIDControllerSetpoint( double position )
    {
        elevatorPIDController.setGoal( position );
        elevatorPIDController.reset( getCurrentElevatorEncoderPosition() );
    }

    public void setPivotPIDControllerSetpoint( double position )
    {
        pivotPIDController.reset( getCurrentPivotEncoderPosition() );
        pivotPIDController.setGoal( position );
    }

    public boolean isElevatorAtSetpoint()
    {
        return elevatorPIDController.atSetpoint();
    }

    public boolean isPivotAtSetpoint()
    {
        return pivotPIDController.atSetpoint();
    }

    public void stopElevator()
    {
        elevatorSparkMax.set(0);
    }

    public void run() {
        SmartDashboard.putNumber( "ElevatorAndPivot/CurrentElevatorEncoder", getCurrentElevatorEncoderPosition() );
        SmartDashboard.putNumber( "ElevatorAndPivot/DesiredElevatorEncoder", elevatorPIDController.getSetpoint().position );
        SmartDashboard.putBoolean( "ElevatorAndPivot/ElevatorAtSetpoint", elevatorPIDController.atSetpoint() );

        SmartDashboard.putNumber( "ElevatorAndPivot/CurrentPivotEncoder", getCurrentPivotEncoderPosition() );
        SmartDashboard.putNumber( "ElevatorAndPivot/DesiredPivotEncoder", pivotPIDController.getSetpoint().position );
        SmartDashboard.putBoolean( "ElevatorAndPivot/PivotAtSetpoint", pivotPIDController.atSetpoint() );

        // If the bottom limit switch is hit, then reset the encoder to 0
        if( !bottomLimitSwitch.get() )
        {
            if( !homed )
            {
                elevatorSparkMax.getEncoder().setPosition(-5);
                stopElevator();
            }
            homed = true;
        }

        if( !homed )
        {
            //startJogElevatorDown();
            elevatorSparkMax.set( -0.05 );
        }
        else
        {
            moveElevatorToDesiredPosition();
        }

        movePivotToDesiredAngle();
    }

}

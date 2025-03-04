package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralAlgaeDevice extends SubsystemBase {
    private DutyCycle rearSensor = new DutyCycle( new DigitalInput(2) );
    private DutyCycle frontSensor = new DutyCycle( new DigitalInput(3) );

    private SparkMax middleMotor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
    private SparkMax topMotor = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
    private SparkMax bottomMotor = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);

    private boolean hasPiece;

    public CoralAlgaeDevice()
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        //config.smartCurrentLimit(5);
        middleMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        topMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        bottomMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
    public void runAlgaeIntake() {
        middleMotor.set(0.5);
        topMotor.set(0.5);
    }

    public void stop() {
        middleMotor.set(0);
        topMotor.set(0);
        bottomMotor.set(0);
    }

    public void runAlgaeDeploy() {
        middleMotor.set(-0.5);
        topMotor.set(-0.5);
    }

    public void runCoralIntake() {
        bottomMotor.set(-0.5);
        middleMotor.set(0.5);
    }

    public void runCoralDeploy() {
        bottomMotor.set(-1);
        middleMotor.set(1);
    }

    public boolean rearSensorHasCoral()
    {
        return rearSensor.getHighTimeNanoseconds() < 1040000;
    }

    public boolean frontSensorHasCoral()
    {
        return frontSensor.getHighTimeNanoseconds() < 1040000;
    }

    public double getTopAlgaeRPM()
    {
        return topMotor.getEncoder().getVelocity();
    }

    public double getBottomAlgaeRPM()
    {
        return middleMotor.getEncoder().getVelocity();
    }


    public void run()
    {
        SmartDashboard.putNumber("CoralAlgaeDevice/RearSensorValue", rearSensor.getHighTimeNanoseconds());
        SmartDashboard.putNumber("CoralAlgaeDevice/FrontSensorValue", frontSensor.getHighTimeNanoseconds());
        SmartDashboard.putBoolean("CoralAlgaeDevice/FrontSensorHasCoral", frontSensorHasCoral());
        SmartDashboard.putBoolean("CoralAlgaeDevice/RearSensorHasCoral", rearSensorHasCoral());

        SmartDashboard.putNumber("CoralAlgaeDevice/TopAlgaeMotorRPM", getTopAlgaeRPM());
        SmartDashboard.putNumber("CoralAlgaeDevice/BottomAlgaeMotorRPM", getBottomAlgaeRPM());
        SmartDashboard.putBoolean("CoralAlgaeDevice/HasPiece", hasPiece || frontSensorHasCoral() && rearSensorHasCoral() );

    }

    public void setHasPiece(boolean hasPiece) {
        this.hasPiece = hasPiece;
    }

    public void runCoralIntakeBackwards() {
        bottomMotor.set(0.5);
        middleMotor.set(-0.5);
    }

    public void publishReadyChecks() {
        SmartDashboard.putBoolean("ReadyChecks/CoralRearSensor", rearSensor.getOutput() > 0);
        SmartDashboard.putBoolean("ReadyChecks/CoralFrontSensor", frontSensor.getOutput() > 0);
    }
}

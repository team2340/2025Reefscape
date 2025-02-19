package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralAlgaeDevice extends SubsystemBase {

    private SparkMax middleMotor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
    private SparkMax topMotor = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
    private SparkMax bottomMotor = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);

    public CoralAlgaeDevice()
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
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
        bottomMotor.set(1);
        middleMotor.set(-1);
    }

    public void runCoralDeploy() {
        bottomMotor.set(-1);
        middleMotor.set(1);
    }
}

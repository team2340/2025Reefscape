package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax climberSparkMax = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);

    public Climber()
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.openLoopRampRate( 0.5 );
        climberSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public Command climberInCommand()
    {
        return Commands.startEnd(
                () -> climberSparkMax.set(-1),
                () -> climberSparkMax.set(0),
                this
        );
    }

    public Command climberOutCommand()
    {
        return Commands.startEnd(
                () -> climberSparkMax.set(1),
                () -> climberSparkMax.set(0),
                this
        );
    }
}

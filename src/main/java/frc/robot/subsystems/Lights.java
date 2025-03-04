package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Lights extends SubsystemBase {
    PWMSparkMax blinken = new PWMSparkMax(0);

    private PATTERN currentPattern = PATTERN.DEFAULT;

    public enum PATTERN {
        INTAKE_CORAL(0.77),
        AUTO_DRIVING(0.65),
        AUTO_DRIVING_REACHED(-0.63),
        DEPLOYING_CORAL(0.93),
        MOVING_ELEVATOR(0.57),
        DISABLED(0.99),
        DEFAULT(-0.69);

        private double pwmValue;
        PATTERN(double values)
        {
            this.pwmValue = values;
        }
    }

    private static Lights instance = null;

    public static Lights getInstance() {
        if( instance == null )
        {
            instance = new Lights();
        }
        return instance;
    }

    private Lights()
    {
        SmartDashboard.putNumber("Lights/LightsOverride", 0);
        instance = this;
    }

    public void setPattern( PATTERN pattern )
    {
        this.currentPattern = pattern;
    }

    public void setDefaultPattern()
    {
        setPattern( PATTERN.DEFAULT );
    }

    public void run()
    {
        if( SmartDashboard.getNumber("Lights/LightsOverride", 0) != 0 )
        {
            blinken.set( SmartDashboard.getNumber("Lights/LightsOverride", 0));
        }
        else
        {
            if( !Robot.getInstance().isEnabled() )
            {
                blinken.set( PATTERN.DISABLED.pwmValue );
            }
            else
            {
                blinken.set(currentPattern.pwmValue);
            }
        }


    }

}

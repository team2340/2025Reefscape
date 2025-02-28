package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.swervedrive.AutoDriving;

public class AprilTagPoseProcessing extends SubsystemBase{
    private AutoDriving autoDriving;

    public AprilTagPoseProcessing(AutoDriving autoDriving )
    {
        this.autoDriving = autoDriving;
    }

    @Override
    public void periodic()
    {
        autoDriving.setDesiredPose();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleJoystick extends Command {
    private XboxController controller;

    public RumbleJoystick(XboxController controller )
    {
        this.controller = controller;
    }

    int count = 0;
    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        count += 1;
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return count > 50;
    }
}

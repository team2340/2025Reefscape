package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class DriverControllerHelpers {

    public static boolean isSlowMode(XboxController controller )
    {
        return controller.getLeftBumperButton() || controller.getRightBumperButton();
    }

    public static double getScaling(XboxController controller )
    {
        if( isSlowMode( controller ) )
        {
            return 0.2;
        }
        else
        {
            return 1.0;
        }
    }

    public static double getX(XboxController controller) {
        if(isRobotCentric(controller)){
            return controller.getRightX() * -1 * getScaling(controller);
        }else{
            return controller.getLeftX() * -1 * getScaling(controller);
        }
    }

    public static double getY(XboxController controller) {
        if(isRobotCentric(controller)) {
            return controller.getRightY() * -1 * getScaling(controller);
        }else{
            return controller.getLeftY() * -1 * getScaling(controller);
        }
    }

    public static double getRotation(XboxController controller)
    {
        return (controller.getLeftTriggerAxis()-controller.getRightTriggerAxis()) * getScaling(controller);
    }
    public static boolean isRobotCentric(XboxController controller) {
        double totalleftstickmovement = Math.abs(controller.getLeftX()) + Math.abs(controller.getLeftY());
        double totalrightstickmovement = Math.abs(controller.getRightX()) + Math.abs(controller.getRightY());
        if(totalleftstickmovement<totalrightstickmovement){
            return true;
        }else{
            return false;
        }
    }
}

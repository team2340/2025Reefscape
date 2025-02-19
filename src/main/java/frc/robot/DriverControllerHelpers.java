package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class DriverControllerHelpers {

    public static double getX(XboxController controller) {
        if(isRobotCentric(controller)){
            return controller.getRightX() * -1;
        }else{
            return controller.getLeftX() * -1;
        }
    }

    public static double getY(XboxController controller) {
        if(isRobotCentric(controller)) {
            return controller.getRightY() * -1;
        }else{
            return controller.getLeftY() * -1;
        }
    }

    public static double getRotation(XboxController controller)
    {

        return controller.getLeftTriggerAxis()-controller.getRightTriggerAxis();
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

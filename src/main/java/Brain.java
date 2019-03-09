
import edu.wpi.first.networktables.NetworkTableEntry;


public class Brain {

    //----------------//
    // Default Values //
    //----------------//

    // Vision - Line Pipeline
    public static double hueMinDefault = 0;
    public static double hueMaxDefault = 180;
    public static double saturationMinDefault = 0;
    public static double saturationMaxDefault = 146;
    public static double valueMinDefault = 232;
    public static double valueMaxDefault = 255;

    // Vision - Front Line Detector
    public static String frontCameraNameDefault = "Front Camera";        
    public static double frontLineContoursDefault = 0;
    public static double frontLineAreaDefault = 0;
    public static double frontLineAngleDefault = 0;
    public static double frontLineXcenterDefault = 0;
    public static double frontLineYcenterDefault = 0;

    // Vision - Left Line Detector
    public static String leftCameraNameDefault = "Left Camera";     
    public static double leftLineContoursDefault = 0;
    public static double leftLineAreaDefault = 0;
    public static double leftLineAngleDefault = 0;
    public static double leftLineXcenterDefault = 0;
    public static double leftLineYcenterDefault = 0;

    // Vision - Right Line Detector
    public static String rightCameraNameDefault = "Right Camera";     
    public static double rightLineContoursDefault = 0;
    public static double rightLineAreaDefault = 0;
    public static double rightLineAngleDefault = 0;
    public static double rightLineXcenterDefault = 0;
    public static double rightLineYcenterDefault = 0;

    //---------------------//
    // NetworkTableEntries //
    //---------------------//

    // Vision - Line Pipeline
    public static NetworkTableEntry hueMinEntry;
    public static NetworkTableEntry hueMaxEntry;
    public static NetworkTableEntry saturationMinEntry;
    public static NetworkTableEntry saturationMaxEntry;
    public static NetworkTableEntry valueMinEntry;
    public static NetworkTableEntry valueMaxEntry;

    // Vision - Front Line Detector
    public static NetworkTableEntry frontCameraNameEntry;
    public static NetworkTableEntry frontLineContoursEntry;
    public static NetworkTableEntry frontLineAreaEntry;
    public static NetworkTableEntry frontLineAngleEntry;
    public static NetworkTableEntry frontLineXcenterEntry;
    public static NetworkTableEntry frontLineYcenterEntry;

    // Vision - Left Line Detector
    public static NetworkTableEntry leftCameraNameEntry;
    public static NetworkTableEntry leftLineContoursEntry;
    public static NetworkTableEntry leftLineAreaEntry;
    public static NetworkTableEntry leftLineAngleEntry;
    public static NetworkTableEntry leftLineXcenterEntry;
    public static NetworkTableEntry leftLineYcenterEntry;

    // Vision - Right Line Detector
    public static NetworkTableEntry rightCameraNameEntry;
    public static NetworkTableEntry rightLineContoursEntry;
    public static NetworkTableEntry rightLineAreaEntry;
    public static NetworkTableEntry rightLineAngleEntry;
    public static NetworkTableEntry rightLineXcenterEntry;
    public static NetworkTableEntry rightLineYcenterEntry;

    //---------//
    // Setters //
    //---------//

    // Vision - Front Line Detector
    public static void setFrontCameraName(String value) {
        frontCameraNameEntry.setString(value);
    }

    public static void setFrontLineContours(double value) {
        frontLineContoursEntry.setDouble(value);
    }

    public static void setFrontLineArea(double value) {
        frontLineAreaEntry.setDouble(value);
    }

    public static void setFrontLineAngle(double value) {
        frontLineAngleEntry.setDouble(value);
    }

    public static void setFrontLineXcenter(double value) {
        frontLineXcenterEntry.setDouble(value);
    }

    public static void setFrontLineYcenter(double value) {
        frontLineYcenterEntry.setDouble(value);
    }

    // Vision - Left Line Detector
    public static void setLeftCameraName(String value) {
        leftCameraNameEntry.setString(value);
    }

    public static void setLeftLineContours(double value) {
        leftLineContoursEntry.setDouble(value);
    }

    public static void setLeftLineArea(double value) {
        leftLineAreaEntry.setDouble(value);
    }

    public static void setLeftLineAngle(double value) {
        leftLineAngleEntry.setDouble(value);
    }

    public static void setLeftLineXcenter(double value) {
        leftLineXcenterEntry.setDouble(value);
    }

    public static void setLeftLineYcenter(double value) {
        leftLineYcenterEntry.setDouble(value);
    }

    // Vision - Right Line Detector
    public static void setRightCameraName(String value) {
        rightCameraNameEntry.setString(value);
    }

    public static void setRightLineContours(double value) {
        rightLineContoursEntry.setDouble(value);
    }

    public static void setRightLineArea(double value) {
        rightLineAreaEntry.setDouble(value);
    }

    public static void setRightLineAngle(double value) {
        rightLineAngleEntry.setDouble(value);
    }

    public static void setRightLineXcenter(double value) {
        rightLineXcenterEntry.setDouble(value);
    }

    public static void setRightLineYcenter(double value) {
        rightLineYcenterEntry.setDouble(value);
    }

    //---------//
    // Getters //
    //---------//

    // Vision - LinePipeline
    public static double getHueMin() {
        return hueMinEntry.getDouble(hueMinDefault);
    }

    public static double getHueMax() {
        return hueMaxEntry.getDouble(hueMaxDefault);
    }

    public static double getSaturationMin() {
        return saturationMinEntry.getDouble(saturationMinDefault);
    }

    public static double getSaturationMax() {
        return saturationMaxEntry.getDouble(saturationMaxDefault);
    }

    public static double getValueMin() {
        return valueMinEntry.getDouble(valueMinDefault);
    }

    public static double getValueMax() {
        return valueMaxEntry.getDouble(valueMaxDefault);
    }
}

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.RaspiPin;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.*;
import org.opencv.imgproc.*;



/*
JSON format:
{
    "team": <team number>,
    "ntmode": <"client" or "server", "client" if unspecified>
    "cameras": [
        {
            "name": <camera name>
            "path": <path, e.g. "/dev/video0">
            "pixel format": <"MJPEG", "YUYV", etc>                      // optional
            "width": <video mode width>                                 // optional
            "height": <video mode height>                               // optional
            "fps": <video mode fps>                                     // optional
            "brightness": <percentage brightness>                       // optional
            "white balance": <"auto", "hold", value>                    // optional
            "exposure": <"auto", "hold", value>                         // optional
            "properties": [                                             // optional
                {
                    "name": <property name>
                    "value": <property value>
                }
            ],
            "stream": {                                                 // optional
                "properties": [
                    {
                        "name": <stream property name>
                        "value": <stream property value>
                    }
                ]
            }
        }
    ]
}
*/

public final class Main {
    private static String configFile = "/boot/frc.json";
    // private static String configFile = "C:\\dev\\SpacePi\\rPi\\frc.json";

    @SuppressWarnings("MemberName")
    public static class CameraConfig {
        public String name;
        public String path;
        public JsonObject config;
        public JsonElement streamConfig;
    }

    public static int team;
    public static boolean server;
    public static List<CameraConfig> cameraConfigs = new ArrayList<>();

    private Main() {
    }

    /**
     * Report parse error.
     */
    public static void parseError(String str) {
        System.err.println("config error in '" + configFile + "': " + str);
    }

    /**
     * Read single camera configuration.
     */
    public static boolean readCameraConfig(JsonObject config) {
        CameraConfig cam = new CameraConfig();

        // name
        JsonElement nameElement = config.get("name");
        if (nameElement == null) {
            parseError("could not read camera name");
            return false;
        }
        cam.name = nameElement.getAsString();

        // path
        JsonElement pathElement = config.get("path");
        if (pathElement == null) {
            parseError("camera '" + cam.name + "': could not read path");
            return false;
        }
        cam.path = pathElement.getAsString();

        // stream properties
        cam.streamConfig = config.get("stream");

        cam.config = config;

        cameraConfigs.add(cam);
        return true;
    }

    /**
     * Read configuration file.
     */
    @SuppressWarnings("PMD.CyclomaticComplexity")
    public static boolean readConfig() {
        // parse file
        JsonElement top;
        try {
            top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
        } catch (IOException ex) {
            System.err.println("could not open '" + configFile + "': " + ex);
            return false;
        }

        // top level must be an object
        if (!top.isJsonObject()) {
            parseError("must be JSON object");
            return false;
        }
        JsonObject obj = top.getAsJsonObject();

        // team number
        JsonElement teamElement = obj.get("team");
        if (teamElement == null) {
            parseError("could not read team number");
            return false;
        }
        team = teamElement.getAsInt();

        // ntmode (optional)
        if (obj.has("ntmode")) {
            String str = obj.get("ntmode").getAsString();
            if ("client".equalsIgnoreCase(str)) {
                server = false;
            } else if ("server".equalsIgnoreCase(str)) {
                server = true;
            } else {
                parseError("could not understand ntmode value '" + str + "'");
            }
        }

        // cameras
        JsonElement camerasElement = obj.get("cameras");
        if (camerasElement == null) {
            parseError("could not read cameras");
            return false;
        }
        JsonArray cameras = camerasElement.getAsJsonArray();
        for (JsonElement camera : cameras) {
            if (!readCameraConfig(camera.getAsJsonObject())) {
                return false;
            }
        }

        return true;
    }

    /**
     * Start running the camera.
     */
    public static VideoSource startCamera(CameraConfig config) {
        System.out.println("Starting camera '" + config.name + "' on " + config.path);
        CameraServer inst = CameraServer.getInstance();
        UsbCamera camera = new UsbCamera(config.name, config.path);
        MjpegServer server = inst.startAutomaticCapture(camera);

        Gson gson = new GsonBuilder().create();

        camera.setConfigJson(gson.toJson(config.config));
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        if (config.streamConfig != null) {
            server.setConfigJson(gson.toJson(config.streamConfig));
        }

        camera.setFPS(30);
        camera.setBrightness(50);
        camera.setExposureManual(40);

        return camera;
    }

    /**
    * LinePipeline class.
    *
    * <p>An OpenCV pipeline generated by GRIP.
    *
    * @author GRIP
    */
    public static class LinePipeline implements VisionPipeline {

        //Outputs
        private Mat hsvThresholdOutput = new Mat();
        private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
        private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

        static {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        }

        /**
         * This is the primary method that runs the entire pipeline and updates the outputs.
         */
            @Override
            public void process(Mat source0) {
            // Step HSV_Threshold0:
            Mat hsvThresholdInput = source0;
                    
            double hsvThresholdHueMin = Brain.getHueMin();
            double hsvThresholdHueMax = Brain.getHueMax();
            double hsvThresholdSaturationMin = Brain.getSaturationMin();
            double hsvThresholdSaturationMax = Brain.getSaturationMax();
            double hsvThresholdValueMin = Brain.getValueMin();
            double hsvThresholdValueMax = Brain.getValueMax();
                    
            double[] hsvThresholdHue = {hsvThresholdHueMin, hsvThresholdHueMax};
            double[] hsvThresholdSaturation = {hsvThresholdSaturationMin, hsvThresholdSaturationMax};
            double[] hsvThresholdValue = {hsvThresholdValueMin, hsvThresholdValueMax};
            
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

            // Step Find_Contours0:
            Mat findContoursInput = hsvThresholdOutput;
            boolean findContoursExternalOnly = false;
            findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

            // Step Filter_Contours0:
            ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
            double filterContoursMinArea = 0.0;
            double filterContoursMinPerimeter = 0.0;
            double filterContoursMinWidth = 20.0;
            double filterContoursMaxWidth = 1000.0;
            double filterContoursMinHeight = 0.0;
            double filterContoursMaxHeight = 1000.0;
            double[] filterContoursSolidity = {0, 100};
            double filterContoursMaxVertices = 1000000.0;
            double filterContoursMinVertices = 0.0;
            double filterContoursMinRatio = 0.0;
            double filterContoursMaxRatio = 1000.0;
                    filterContours(filterContoursContours,
                                    filterContoursMinArea,
                                    filterContoursMinPerimeter,
                                    filterContoursMinWidth,
                                    filterContoursMaxWidth,
                                    filterContoursMinHeight,
                                    filterContoursMaxHeight,
                                    filterContoursSolidity,
                                    filterContoursMaxVertices,
                                    filterContoursMinVertices,
                                    filterContoursMinRatio,
                                    filterContoursMaxRatio,
                                    filterContoursOutput);
        }

        /**
         * This method is a generated getter for the output of a HSV_Threshold.
         * @return Mat output from HSV_Threshold.
         */
        public Mat hsvThresholdOutput() {
            return hsvThresholdOutput;
        }

        /**
         * This method is a generated getter for the output of a Find_Contours.
         * @return ArrayList<MatOfPoint> output from Find_Contours.
         */
        public ArrayList<MatOfPoint> findContoursOutput() {
            return findContoursOutput;
        }

        /**
         * This method is a generated getter for the output of a Filter_Contours.
         * @return ArrayList<MatOfPoint> output from Filter_Contours.
         */
        public ArrayList<MatOfPoint> filterContoursOutput() {
            return filterContoursOutput;
        }


        /**
         * Segment an image based on hue, saturation, and value ranges.
         *
         * @param input The image on which to perform the HSL threshold.
         * @param hue The min and max hue
         * @param sat The min and max saturation
         * @param val The min and max value
         * @param output The image in which to store the output.
         */
        private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
            Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
        }

        /**
         * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
         * @param input The image on which to perform the Distance Transform.
         * @param type The Transform.
         * @param maskSize the size of the mask.
         * @param output The image in which to store the output.
         */
        private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            }
            else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);
        }


        /**
         * Filters out contours that do not meet certain criteria.
         * @param inputContours is the input list of contours
         * @param output is the the output list of contours
         * @param minArea is the minimum area of a contour that will be kept
         * @param minPerimeter is the minimum perimeter of a contour that will be kept
         * @param minWidth minimum width of a contour
         * @param maxWidth maximum width
         * @param minHeight minimum height
         * @param maxHeight maximimum height
         * @param Solidity the minimum and maximum solidity of a contour
         * @param minVertexCount minimum vertex Count of the contours
         * @param maxVertexCount maximum vertex Count
         * @param minRatio minimum ratio of width to height
         * @param maxRatio maximum ratio of width to height
         */
            private void filterContours(List<MatOfPoint> inputContours,
                                        double minArea,
                                        double minPerimeter,
                                        double minWidth,
                                        double maxWidth,
                                        double minHeight,
                                        double maxHeight,
                                        double[] solidity,
                                        double maxVertexCount,
                                        double minVertexCount,
                                        double minRatio,
                                        double maxRatio,
                                        List<MatOfPoint> output) {
            final MatOfInt hull = new MatOfInt();
            output.clear();
            //operation
            for (int i = 0; i < inputContours.size(); i++) {
                final MatOfPoint contour = inputContours.get(i);
                final Rect bb = Imgproc.boundingRect(contour);
                if (bb.width < minWidth || bb.width > maxWidth) continue;
                if (bb.height < minHeight || bb.height > maxHeight) continue;
                final double area = Imgproc.contourArea(contour);
                if (area < minArea) continue;
                if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++) {
                    int index = (int)hull.get(j, 0)[0];
                    double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                final double solid = 100 * area / Imgproc.contourArea(mopHull);
                if (solid < solidity[0] || solid > solidity[1]) continue;
                if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)    continue;
                final double ratio = bb.width / (double)bb.height;
                if (ratio < minRatio || ratio > maxRatio) continue;
                output.add(contour);
            }
        }
    }

    public static class Brain {

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
        public static double frontPiTime = 0;

        // Vision - Left Line Detector
        public static String leftCameraNameDefault = "Left Camera";     
        public static double leftLineContoursDefault = 0;
        public static double leftLineAreaDefault = 0;
        public static double leftLineAngleDefault = 0;
        public static double leftLineXcenterDefault = 0;
        public static double leftLineYcenterDefault = 0;
        public static double leftPiTime = 0;

        // Vision - Right Line Detector
        public static String rightCameraNameDefault = "Right Camera";     
        public static double rightLineContoursDefault = 0;
        public static double rightLineAreaDefault = 0;
        public static double rightLineAngleDefault = 0;
        public static double rightLineXcenterDefault = 0;
        public static double rightLineYcenterDefault = 0;
        public static double rightPiTime = 0;

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
        public static NetworkTableEntry frontPiTimeEntry;

        // Vision - Left Line Detector
        public static NetworkTableEntry leftCameraNameEntry;
        public static NetworkTableEntry leftLineContoursEntry;
        public static NetworkTableEntry leftLineAreaEntry;
        public static NetworkTableEntry leftLineAngleEntry;
        public static NetworkTableEntry leftLineXcenterEntry;
        public static NetworkTableEntry leftLineYcenterEntry;
        public static NetworkTableEntry leftPiTimeEntry;

        // Vision - Right Line Detector
        public static NetworkTableEntry rightCameraNameEntry;
        public static NetworkTableEntry rightLineContoursEntry;
        public static NetworkTableEntry rightLineAreaEntry;
        public static NetworkTableEntry rightLineAngleEntry;
        public static NetworkTableEntry rightLineXcenterEntry;
        public static NetworkTableEntry rightLineYcenterEntry;
        public static NetworkTableEntry rightPiTimeEntry;

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

        public static void setFrontPiTime(double value) {
            frontPiTimeEntry.setDouble(value);
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

        public static void setLeftPiTime(double value) {
            leftPiTimeEntry.setDouble(value);
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

        public static void setRightPiTime(double value) {
            rightPiTimeEntry.setDouble(value);
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

    public enum Quadrant {
        UPPERLEFT, UPPERRIGHT, LOWERLEFT, LOWERRIGHT;

        public static int totalHeight = 160;
        public static int totalWidth = 120;

        public static Quadrant getQuadrant(double x, double y) {
            boolean isUpper = (y <= totalHeight / 2);
            boolean isLeft = (x <= totalWidth / 2);
            if (isUpper) {
                if (isLeft) {
                    return UPPERLEFT;
                }
                else {
                    return UPPERRIGHT;
                }
            }
            else {
                if (isLeft) {
                    return LOWERLEFT;
                }
                else {
                    return LOWERRIGHT;
                }
            }
        }
    }

    public enum CameraPosition {
        FRONT, LEFT, RIGHT
    }

    public static void startLineDetection(VideoSource cam, double minimumArea, CameraPosition camPosition) {
        Timer piTimer = new Timer();
        piTimer.reset();
        piTimer.start();

        VisionThread visionThread = new VisionThread(cam, new LinePipeline(), pipeline -> {
            double elapsedTime = piTimer.get();
            switch (camPosition) {
                case FRONT:
                    Brain.setFrontCameraName("Front");
                    Brain.setFrontPiTime(elapsedTime);
                case LEFT:
                    Brain.setFrontCameraName("Left");
                    Brain.setLeftPiTime(elapsedTime);
                case RIGHT:
                    Brain.setFrontCameraName("Right");
                    Brain.setRightPiTime(elapsedTime);
            }

            ArrayList<MatOfPoint> output = pipeline.filterContoursOutput();
            int outputSize = output.size();
            switch (camPosition) {
                case FRONT:
                    Brain.setFrontLineContours(outputSize);
                case LEFT:
                    Brain.setLeftLineContours(outputSize);
                case RIGHT:
                    Brain.setRightLineContours(outputSize);
            }
            // We can only work with one contour
            if (outputSize == 1) {
                String camName = cam.getName();
                // System.out.println(elapsedTime + " : " + camName + " -> One contour identified, checking minimum size...");
                MatOfPoint contour = output.get(0);

                // Get the rotated rectangle
                Point[] points = contour.toArray();
                MatOfPoint2f contour2f = new MatOfPoint2f(points);
                RotatedRect rotRect = Imgproc.minAreaRect(contour2f);

                // Get the area of the rotated rectangle
                double area = rotRect.size.area();
                if (area >= minimumArea) {
                    // Get the center X & Y of the bounding rectangle
                    Rect boundRect = rotRect.boundingRect();
                    double centerX = boundRect.x + (boundRect.width / 2);
                    double centerY = boundRect.y + (boundRect.height / 2);

                    // Get the rotation angle of the rotated rectangle
                    double angle = rotRect.angle;
                    if (rotRect.size.width < rotRect.size.height) {
                        angle = 90 + angle;
                    }
                    Quadrant centerQuad = Quadrant.getQuadrant(centerX, centerY);
                    switch (centerQuad) {
                        case UPPERLEFT:
                                break;
                        case UPPERRIGHT:
                                break;
                        case LOWERLEFT:
                                if (angle > 0) angle = angle - 180;
                                break;
                        case LOWERRIGHT:
                                if (angle < 0) angle = angle + 180;
                                break;
                    }

                    // Add the values to NetworkTables via the Brain
                    switch (camPosition) {
                        case FRONT:
                            Brain.setFrontLineArea(area);
                            Brain.setFrontLineAngle(angle);
                            Brain.setFrontLineXcenter(centerX);
                            Brain.setFrontLineYcenter(centerY);
                        case LEFT:
                            Brain.setLeftLineArea(area);
                            Brain.setLeftLineAngle(angle);
                            Brain.setLeftLineXcenter(centerX);
                            Brain.setLeftLineYcenter(centerY);
                        case RIGHT:
                            Brain.setRightLineArea(area);
                            Brain.setRightLineAngle(angle);
                            Brain.setRightLineXcenter(centerX);
                            Brain.setRightLineYcenter(centerY);
                    }
                    System.out.println(elapsedTime + " : " + camName + " -> Line Detected!");
                }
            }
            else {
                // We can't work with these contours, so set everything to default
                switch (camPosition) {
                    case FRONT:
                        Brain.setFrontLineArea(Brain.frontLineAreaDefault);
                        Brain.setFrontLineAngle(Brain.frontLineAngleDefault);
                        Brain.setFrontLineXcenter(Brain.frontLineXcenterDefault);
                        Brain.setFrontLineYcenter(Brain.frontLineYcenterDefault);
                    case LEFT:
                        Brain.setLeftLineArea(Brain.leftLineAreaDefault);
                        Brain.setLeftLineAngle(Brain.leftLineAngleDefault);
                        Brain.setLeftLineXcenter(Brain.leftLineXcenterDefault);
                        Brain.setLeftLineYcenter(Brain.leftLineYcenterDefault);
                    case RIGHT:
                        Brain.setRightLineArea(Brain.rightLineAreaDefault);
                        Brain.setRightLineAngle(Brain.rightLineAngleDefault);
                        Brain.setRightLineXcenter(Brain.rightLineXcenterDefault);
                        Brain.setRightLineYcenter(Brain.rightLineYcenterDefault);
                }

                // TODO: consider checking all the contours, and if only one meets the minimum area requirements, use that
            }
        });
        visionThread.start();
    }

    public static class DistanceMonitor {
    
        private final static float SOUND_SPEED = 340.29f;  // speed of sound in m/s
        
        private final static int TRIG_DURATION_IN_MICROS = 10; // trigger duration of 10 micro s
        private final static int WAIT_DURATION_IN_MILLIS = 60; // wait 60 milli s
    
        private final static int TIMEOUT = 2100;
        
        private final static GpioController gpio = GpioFactory.getInstance();
        
        private final GpioPinDigitalInput echoPin;
        private final GpioPinDigitalOutput trigPin;
                
        private DistanceMonitor( Pin echoPin, Pin trigPin ) {
            this.echoPin = gpio.provisionDigitalInputPin( echoPin );
            this.trigPin = gpio.provisionDigitalOutputPin( trigPin );
            this.trigPin.low();
        }
        
        /*
         * This method returns the distance measured by the sensor in cm
         * 
         * @throws TimeoutException if a timeout occurs
         */
        public float measureDistance() throws TimeoutException {
            this.triggerSensor();
            this.waitForSignal();
            long duration = this.measureSignal();
            
            return duration * SOUND_SPEED / ( 2 * 10000 );
        }
    
        /**
         * Put a high on the trig pin for TRIG_DURATION_IN_MICROS
         */
        private void triggerSensor() {
            try {
                this.trigPin.high();
                Thread.sleep( 0, TRIG_DURATION_IN_MICROS * 1000 );
                this.trigPin.low();
            } catch (InterruptedException ex) {
                System.err.println( "Interrupt during trigger" );
            }
        }
        
        /**
         * Wait for a high on the echo pin
         * 
         * @throws DistanceMonitor.TimeoutException if no high appears in time
         */
        private void waitForSignal() throws TimeoutException {
            int countdown = TIMEOUT;
            
            while( this.echoPin.isLow() && countdown > 0 ) {
                countdown--;
            }
            
            if( countdown <= 0 ) {
                throw new TimeoutException( "Timeout waiting for signal start" );
            }
        }
        
        /**
         * @return the duration of the signal in micro seconds
         * @throws DistanceMonitor.TimeoutException if no low appears in time
         */
        private long measureSignal() throws TimeoutException {
            int countdown = TIMEOUT;
            long start = System.nanoTime();
            while( this.echoPin.isHigh() && countdown > 0 ) {
                countdown--;
            }
            long end = System.nanoTime();
            
            if( countdown <= 0 ) {
                throw new TimeoutException( "Timeout waiting for signal end" );
            }
            
            return (long)Math.ceil( ( end - start ) / 1000.0 );  // Return micro seconds
        }

    }

    private static class TimeoutException extends Exception {

        private final String reason;
        
        public TimeoutException( String reason ) {
            this.reason = reason;
        }
        
        @Override
        public String toString() {
            return this.reason;
        }
    }

    /**
     * Main.
     */
    public static void main(String... args) {
        if (args.length > 0) {
            configFile = args[0];
        }

        // read configuration
        if (!readConfig()) {
            return;
        }

        // start NetworkTables
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        if (server) {
            System.out.println("Setting up NetworkTables server");
            ntinst.startServer();
        } else {
            System.out.println("Setting up NetworkTables client for team " + team);
            ntinst.startClientTeam(team);
        }

        NetworkTable hsvTable = ntinst.getTable("Shuffleboard/Vision/HSV Thresholds");
        Brain.hueMinEntry = hsvTable.getEntry("Hue Minimum");
        Brain.hueMaxEntry = hsvTable.getEntry("Hue Maximum");
        Brain.saturationMinEntry = hsvTable.getEntry("Saturation Minimum");
        Brain.saturationMaxEntry = hsvTable.getEntry("Saturation Maximum");
        Brain.valueMinEntry = hsvTable.getEntry("Value Minimum");
        Brain.valueMaxEntry = hsvTable.getEntry("Value Maximum");

        NetworkTable frontCameraTable = ntinst.getTable("Shuffleboard/Vision/Front Camera");
        Brain.frontCameraNameEntry = frontCameraTable.getEntry("Front Camera Name");
        Brain.frontLineContoursEntry = frontCameraTable.getEntry("Front Line Contours");
        Brain.frontLineAreaEntry = frontCameraTable.getEntry("Front Line Area");
        Brain.frontLineAngleEntry = frontCameraTable.getEntry("Front Line Angle");
        Brain.frontLineXcenterEntry = frontCameraTable.getEntry("Front Line Center X");
        Brain.frontLineYcenterEntry = frontCameraTable.getEntry("Front Line Center Y");
        Brain.frontPiTimeEntry = frontCameraTable.getEntry("Front Pi Time");

        NetworkTable leftCameraTable = ntinst.getTable("Shuffleboard/Vision/Left Camera");
        Brain.leftCameraNameEntry = leftCameraTable.getEntry("Left Camera Name");
        Brain.leftLineContoursEntry = leftCameraTable.getEntry("Left Line Contours");
        Brain.leftLineAreaEntry = leftCameraTable.getEntry("Left Line Area");
        Brain.leftLineAngleEntry = leftCameraTable.getEntry("Left Line Angle");
        Brain.leftLineXcenterEntry = leftCameraTable.getEntry("Left Line Center X");
        Brain.leftLineYcenterEntry = leftCameraTable.getEntry("Left Line Center Y");
        Brain.leftPiTimeEntry = leftCameraTable.getEntry("Left Pi Time");

        NetworkTable rightCameraTable = ntinst.getTable("Shuffleboard/Vision/Right Camera");
        Brain.rightCameraNameEntry = rightCameraTable.getEntry("Right Camera Name");
        Brain.rightLineContoursEntry = rightCameraTable.getEntry("Right Line Contours");
        Brain.rightLineAreaEntry = rightCameraTable.getEntry("Right Line Area");
        Brain.rightLineAngleEntry = rightCameraTable.getEntry("Right Line Angle");
        Brain.rightLineXcenterEntry = rightCameraTable.getEntry("Right Line Center X");
        Brain.rightLineYcenterEntry = rightCameraTable.getEntry("Right Line Center Y");
        Brain.rightPiTimeEntry = rightCameraTable.getEntry("Right Pi Time");

        // start cameras
        List<VideoSource> cameras = new ArrayList<>();
        for (CameraConfig cameraConfig : cameraConfigs) {
            cameras.add(startCamera(cameraConfig));
        }

        // start image processing on camera 0 if present
        double minimumArea = (Quadrant.totalHeight / 3) ^ 2;
        System.out.println("Minimum Area: " + minimumArea);
        int numOfCameras = cameras.size();
        System.out.println("Number of cameras: " + numOfCameras);
        startLineDetection(cameras.get(0), minimumArea, CameraPosition.FRONT);
        startLineDetection(cameras.get(1), minimumArea, CameraPosition.LEFT);
        startLineDetection(cameras.get(2), minimumArea, CameraPosition.RIGHT);

        // TODO: Need to figure out how to install this on the raspberry pi:
        // http://wiringpi.com/download-and-install/

        // // start the distance sensor
        // Pin echoPin = RaspiPin.GPIO_20; // PI4J custom numbering (pin 20)
        // Pin trigPin = RaspiPin.GPIO_18; // PI4J custom numbering (pin 18)
        // DistanceMonitor monitor = new DistanceMonitor( echoPin, trigPin );

        // loop forever
        for (;;) {
            // try {
            //     System.out.printf( "%1$d,%2$.3f%n", System.currentTimeMillis(), monitor.measureDistance() );
            // }
            // catch( TimeoutException e ) {
            //     System.err.println( e );
            // }
            try {
                Thread.sleep(10000);
            } catch (InterruptedException ex) {
                return;
            }
        }
    }
}

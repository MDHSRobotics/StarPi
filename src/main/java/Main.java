
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

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import org.opencv.core.Mat;

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
    // private static String configFile = "C:\\dev\\SpaceBot\\rPi\\frc.json";

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

        return camera;
    }

    // /**
    //    * Example pipeline.
    //    */
    // public static class MyPipeline implements VisionPipeline {
    //     public int val;

    //     @Override
    //     public void process(Mat mat) {
    //         val += 1;
    //     }
    // }

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

        // Vision - LinePipeline
        public static double hueMinDefault = 0;
        public static double hueMaxDefault = 180;
        public static double saturationMinDefault = 0;
        public static double saturationMaxDefault = 146;
        public static double valueMinDefault = 232;
        public static double valueMaxDefault = 255;

        // Vision - LineDetector
        public static double hatchLineAreaDefault = 0;
        public static double hatchLineAngleDefault = 0;
        public static double hatchLineXcenterDefault = 0;
        public static double hatchLineYcenterDefault = 0;
        public static boolean hatchLineDetectedDefault = false;

        //---------------------//
        // NetworkTableEntries //
        //---------------------//

        // Vision - LinePipeline
        public static NetworkTableEntry hueMinEntry;
        public static NetworkTableEntry hueMaxEntry;
        public static NetworkTableEntry saturationMinEntry;
        public static NetworkTableEntry saturationMaxEntry;
        public static NetworkTableEntry valueMinEntry;
        public static NetworkTableEntry valueMaxEntry;

        // Vision - LineDetector
        public static NetworkTableEntry hatchLineAreaEntry;
        public static NetworkTableEntry hatchLineAngleEntry;
        public static NetworkTableEntry hatchLineXcenterEntry;
        public static NetworkTableEntry hatchLineYcenterEntry;
        public static NetworkTableEntry hatchLineDetectedEntry;

        //---------//
        // Setters //
        //---------//

        // Vision - LinePipeline
        public static void setHueMin(NetworkTableEntry entry) {
            double value = entry.getDouble(hueMinDefault);
            hueMinEntry.setDouble(value);
        }

        public static void setHueMax(NetworkTableEntry entry) {
            double value = entry.getDouble(hueMaxDefault);
            hueMaxEntry.setDouble(value);
        }

        public static void setSaturationMin(NetworkTableEntry entry) {
            double value = entry.getDouble(saturationMinDefault);
            saturationMinEntry.setDouble(value);
        }

        public static void setSaturationMax(NetworkTableEntry entry) {
            double value = entry.getDouble(saturationMaxDefault);
            saturationMaxEntry.setDouble(value);
        }

        public static void setValueMin(NetworkTableEntry entry) {
            double value = entry.getDouble(valueMinDefault);
            valueMinEntry.setDouble(value);
        }

        public static void setValueMax(NetworkTableEntry entry) {
            double value = entry.getDouble(valueMaxDefault);
            valueMaxEntry.setDouble(value);
        }

        // Vision - LineDetector
        public static void setHatchLineArea(NetworkTableEntry entry) {
            double value = entry.getDouble(hatchLineAreaDefault);
            hatchLineAreaEntry.setDouble(value);
        }

        public static void setHatchLineArea(double value) {
            hatchLineAreaEntry.setDouble(value);
        }

        public static void setHatchLineAngle(NetworkTableEntry entry) {
            double value = entry.getDouble(hatchLineAngleDefault);
            hatchLineAngleEntry.setDouble(value);
        }

        public static void setHatchLineAngle(double value) {
            hatchLineAngleEntry.setDouble(value);
        }

        public static void setHatchLineXcenter(NetworkTableEntry entry) {
            double value = entry.getDouble(hatchLineXcenterDefault);
            hatchLineXcenterEntry.setDouble(value);
        }

        public static void setHatchLineXcenter(double value) {
            hatchLineXcenterEntry.setDouble(value);
        }

        public static void setHatchLineYcenter(NetworkTableEntry entry) {
            double value = entry.getDouble(hatchLineYcenterDefault);
            hatchLineYcenterEntry.setDouble(value);
        }

        public static void setHatchLineYcenter(double value) {
            hatchLineYcenterEntry.setDouble(value);
        }

        public static void setHatchLineDetected() {
            boolean detected = Robot.robotLineDetectorHatch.lineDetected();
            hatchLineDetectedEntry.setBoolean(detected);
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

        NetworkTable shuffleboardTable = ntinst.getTable("Shuffleboard");
        NetworkTable visionTable = shuffleboardTable.getTable("Vision");
        hueMinEntry = visionTable.getEntry("Hue Minimum");
        hueMaxEntry = visionTable.getEntry("Hue Maximum");
        saturationMinEntry = visionTable.getEntry("Saturation Minimum");
        saturationMaxEntry = visionTable.getEntry("Saturation Maximum");
        valueMinEntry = visionTable.getEntry("Value Minimum");
        valueMaxEntry = visionTable.getEntry("Value Maximum");

        // start cameras
         List<VideoSource> cameras = new ArrayList<>();
        for (CameraConfig cameraConfig : cameraConfigs) {
            cameras.add(startCamera(cameraConfig));
        }

        // start image processing on camera 0 if present
        if (cameras.size() >= 1) {
            VisionThread visionThread = new VisionThread(cameras.get(0), new LinePipeline(), pipeline -> {
                ArrayList<MatOfPoint> output = pipeline.filterContoursOutput();
                int outputSize = output.size();
                // We can only work with one contour
                if (outputSize == 1) {
                    System.out.println("One contour identified, checking minimum size...");
                    MatOfPoint contour = output.get(0);

                    // Get the rotated rectangle
                    Point[] points = contour.toArray();
                    MatOfPoint2f contour2f = new MatOfPoint2f(points);
                    RotatedRect rotRect = Imgproc.minAreaRect(contour2f);

                    // Get the area of the rotated rectangle
                    double area = rotRect.size.area();
                    if (area >= m_minimumArea) {
                        // Get the center X & Y of the bounding rectangle
                        Rect boundRect = rotRect.boundingRect();
                        double centerX = boundRect.x + (boundRect.width / 2);
                        double centerY = boundRect.y + (boundRect.height / 2);

                        // Get the rotation angle of the rotated rectangle
                        double angle = rotRect.angle;
                        if (rotRect.size.width < rotRect.size.height) {
                            angle = 90 + angle;
                        }
                        // Quadrant centerQuad = getQuadrant(centerX, centerY);
                        // switch (centerQuad) {
                        //     case UPPERLEFT:
                        //             break;
                        //     case UPPERRIGHT:
                        //             break;
                        //     case LOWERLEFT:
                        //             if (angle > 0) angle = angle - 180;
                        //             break;
                        //     case LOWERRIGHT:
                        //             if (angle < 0) angle = angle + 180;
                        //             break;
                        // }

                        // Add the values to NetworkTables via the Brain
                        Brain.setHatchLineArea(area);
                        Brain.setHatchLineAngle(angle);
                        Brain.setHatchLineXcenter(centerX);
                        Brain.setHatchLineYcenter(centerY);

                        System.out.println("Line Detected!");
                    }
                }
                else {
                    // We can't work with these contours, so set everything to default
                    Brain.setHatchLineArea(Brain.hatchLineAreaDefault);
                    Brain.setHatchLineAngle(Brain.hatchLineAngleDefault);
                    Brain.setHatchLineXcenter(Brain.hatchLineXcenterDefault);
                    Brain.setHatchLineYcenter(Brain.hatchLineYcenterDefault);

                    // TODO: consider checking all the contours, and if only one meets the minimum area requirements, use that
                }
            });
            /* something like this for GRIP:
            VisionThread visionThread = new VisionThread(cameras.get(0),
                            new GripPipeline(), pipeline -> {
                ...
            });
             */
            visionThread.start();
        }

        // loop forever
        for (;;) {
            try {
                Thread.sleep(10000);
            } catch (InterruptedException ex) {
                return;
            }
        }
    }
}

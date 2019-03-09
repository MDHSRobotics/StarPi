
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
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


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
    "switched cameras": [
       {
           "name": <virtual camera name>
           "key": <network table key used for selection>
           // if NT value is a string, it's treated as a name
           // if NT value is a double, it's treated as an integer index
       }
   ]
}
*/

public final class Main {
    private static String configFile = "/boot/frc.json";

    @SuppressWarnings("MemberName")
    public static class CameraConfig {
        public String name;
        public String path;
        public JsonObject config;
        public JsonElement streamConfig;
    }

    @SuppressWarnings("MemberName")
    public static class SwitchedCameraConfig {
        public String name;
        public String key;
    };

    public static int team;
    public static boolean server;
    public static List<CameraConfig> cameraConfigs = new ArrayList<>();
    public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
    public static List<VideoSource> cameras = new ArrayList<>();

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
     * Read single switched camera configuration.
     */
    public static boolean readSwitchedCameraConfig(JsonObject config) {
        SwitchedCameraConfig cam = new SwitchedCameraConfig();

        // name
        JsonElement nameElement = config.get("name");
        if (nameElement == null) {
            parseError("could not read switched camera name");
            return false;
        }
        cam.name = nameElement.getAsString();

        // path
        JsonElement keyElement = config.get("key");
        if (keyElement == null) {
            parseError("switched camera '" + cam.name + "': could not read key");
            return false;
        }
        cam.key = keyElement.getAsString();

        switchedCameraConfigs.add(cam);
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

        if (obj.has("switched cameras")) {
            JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
            for (JsonElement camera : switchedCameras) {
                if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
                    return false;
                }
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
     * Start running the switched camera.
     */
    public static MjpegServer startSwitchedCamera(SwitchedCameraConfig config) {
        System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
        MjpegServer server = CameraServer.getInstance().addSwitchedCamera(config.name);

        NetworkTableInstance.getDefault()
            .getEntry(config.key)
            .addListener(event -> {
                  if (event.value.isDouble()) {
                      int i = (int) event.value.getDouble();
                      if (i >= 0 && i < cameras.size()) {
                          server.setSource(cameras.get(i));
                      }
                  } else if (event.value.isString()) {
                      String str = event.value.getString();
                      for (int i = 0; i < cameraConfigs.size(); i++) {
                          if (str.equals(cameraConfigs.get(i).name)) {
                              server.setSource(cameras.get(i));
                              break;
                          }
                      }
                 }
            },
            EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        return server;
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
        Brain.frontLineContoursEntry = frontCameraTable.getEntry("Front Line Contours");
        Brain.frontLineAreaEntry = frontCameraTable.getEntry("Front Line Area");
        Brain.frontLineAngleEntry = frontCameraTable.getEntry("Front Line Angle");
        Brain.frontLineXcenterEntry = frontCameraTable.getEntry("Front Line Center X");
        Brain.frontLineYcenterEntry = frontCameraTable.getEntry("Front Line Center Y");

        NetworkTable leftCameraTable = ntinst.getTable("Shuffleboard/Vision/Left Camera");
        Brain.leftLineContoursEntry = leftCameraTable.getEntry("Left Line Contours");
        Brain.leftLineAreaEntry = leftCameraTable.getEntry("Left Line Area");
        Brain.leftLineAngleEntry = leftCameraTable.getEntry("Left Line Angle");
        Brain.leftLineXcenterEntry = leftCameraTable.getEntry("Left Line Center X");
        Brain.leftLineYcenterEntry = leftCameraTable.getEntry("Left Line Center Y");

        NetworkTable rightCameraTable = ntinst.getTable("Shuffleboard/Vision/Right Camera");
        Brain.rightLineContoursEntry = rightCameraTable.getEntry("Right Line Contours");
        Brain.rightLineAreaEntry = rightCameraTable.getEntry("Right Line Area");
        Brain.rightLineAngleEntry = rightCameraTable.getEntry("Right Line Angle");
        Brain.rightLineXcenterEntry = rightCameraTable.getEntry("Right Line Center X");
        Brain.rightLineYcenterEntry = rightCameraTable.getEntry("Right Line Center Y");

        // start cameras
        for (CameraConfig config : cameraConfigs) {
            cameras.add(startCamera(config));
        }

        // start switched cameras
        for (SwitchedCameraConfig config : switchedCameraConfigs) {
            startSwitchedCamera(config);
        }

        // start image processing on camera 0 if present
        int numOfCameras = cameras.size();
        System.out.println("Number of cameras: " + numOfCameras);
        Vision frontVis = new Vision(Vision.CameraPosition.FRONT);
        Vision leftVis = new Vision(Vision.CameraPosition.LEFT);
        Vision rightVis = new Vision(Vision.CameraPosition.RIGHT);
        frontVis.startLineDetection(cameras.get(0));
        leftVis.startLineDetection(cameras.get(1));
        rightVis.startLineDetection(cameras.get(2));

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


import java.util.ArrayList;

import edu.wpi.cscore.VideoSource;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.*;
import org.opencv.imgproc.*;


public class Vision {

    public enum CameraPosition {
        FRONT, LEFT, RIGHT
    }

    public enum Quadrant {
        UPPERLEFT, UPPERRIGHT, LOWERLEFT, LOWERRIGHT;

        public static int totalHeight = 320;
        public static int totalWidth = 240;

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

    public CameraPosition camPosition;

    public Vision(CameraPosition position) {
        this.camPosition = position;
    }

    public void startLineDetection(VideoSource cam) {
        Timer piTimer = new Timer();
        piTimer.reset();
        piTimer.start();

        double minimumArea = (Vision.Quadrant.totalHeight / 3) ^ 2;

        VisionThread visionThread = new VisionThread(cam, new LinePipeline(), pipeline -> {
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
                    String camName = cam.getName();
                    double elapsedTime = piTimer.get();
                    System.out.println(camName + " -> Line Detected! : " + elapsedTime);
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

}

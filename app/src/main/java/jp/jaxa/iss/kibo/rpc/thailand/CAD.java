package jp.jaxa.iss.kibo.rpc.thailand;

import android.graphics.RectF;
import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class CAD {




    public static List<Object> Centroid_processing(List<RectF> a, int post_num, List<Float> score) {
        int bypass = 0;
        class Utils {
            public double intersectionArea(int[] rect1, int[] rect2) {
                int xLeft = Math.max(rect1[0], rect2[0]);
                int yTop = Math.max(rect1[1], rect2[1]);
                int xRight = Math.min(rect1[2], rect2[2]);
                int yBottom = Math.min(rect1[3], rect2[3]);

                if (xRight < xLeft || yBottom < yTop) {
                    return 0.0;
                }

                return (xRight - xLeft) * (yBottom - yTop);
            }
        }
        Utils utils = new Utils();
        String default_file;

        default_file = "sdcard/data/jp.jaxa.iss.kibo.rpc.thailand/immediate/DebugImages/post_" + post_num + ".png";
        Mat src = Imgcodecs.imread(default_file, Imgcodecs.IMREAD_COLOR);
        Mat img_show = src.clone();
        List<Integer> faultyRectangles = new ArrayList<>();
        List<Integer> correctRectangles = new ArrayList<>();

        for (int n = 0; n < a.size(); n++) {
            RectF rectF = a.get(n);
            Rect roi = new Rect((int) rectF.left, (int) rectF.top, (int) (rectF.right - rectF.left), (int) (rectF.bottom - rectF.top));
            Mat croppedImg = new Mat(src, roi);
            Mat grayImage = new Mat();
            Imgproc.cvtColor(croppedImg, grayImage, Imgproc.COLOR_BGR2GRAY);
            Mat edged = new Mat();
            Imgproc.Canny(grayImage, edged, 50, 150);

            Mat kernel2 = Mat.ones(2, 2, CvType.CV_8U);
            Mat dilation = new Mat();
            Imgproc.dilate(edged, dilation, kernel2);

            Mat binaryImage = new Mat();
            Imgproc.threshold(dilation, binaryImage, 170, 255, Imgproc.THRESH_BINARY);

            // Find contours in the binary image
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(binaryImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double totalFilledArea = 0;
            for (MatOfPoint contour : contours) {
                totalFilledArea += Imgproc.contourArea(contour);
            }

            // Calculate total area of the cropped image
            double imageArea = roi.width * roi.height;

            // Calculate percentage of filled area
            double percentageFilled = (totalFilledArea / imageArea) * 100;
            Log.i("CAD", "Filled_Precentage:" + percentageFilled);
            double totalArea = 0;
            double weightedSumX = 0;
            double weightedSumY = 0;

            for (MatOfPoint contour : contours) {
                Moments moments = Imgproc.moments(contour);
                double area = moments.get_m00();
                if (area != 0) {
                    double cx = moments.get_m10() / area;
                    double cy = moments.get_m01() / area;
                    totalArea += area;
                    weightedSumX += cx * area;
                    weightedSumY += cy * area;
                }
            }

            if (totalArea != 0) {
                double centroidX = weightedSumX / totalArea;
                double centroidY = weightedSumY / totalArea;
                Log.i("CAD", "Centroid of all contours in Rectangle:" + centroidX + "," + centroidY);
                double centerX = roi.width / 2.0;
                double centerY = roi.height / 2.0;
                double deviationX = ((centroidX - centerX) / roi.width) * 100;
                double deviationY = ((centroidY - centerY) / roi.height) * 100;
                double sumOfAbsolutes = Math.abs(deviationX) + Math.abs(deviationY);
                Log.i("CAD", "Deviation from center in Rectangle:" + deviationX + "," + deviationY);
                Log.i("CAD", "Sum of absolutes:" + sumOfAbsolutes);
                if (score.get(n) >= 0.83) {
                    sumOfAbsolutes = 1;
                    bypass=1;

                }
                if (sumOfAbsolutes >= 12.5) {
                    // Mark rectangle as faulty (red)
                    Imgproc.rectangle(img_show, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y + roi.height), new Scalar(255, 0, 0), 2);
                    Log.i("Fault detected in Rectangle:", String.valueOf(n + 1));
                    // Add this rectangle to faultyRectangles
                    faultyRectangles.add(n);
                } else {
                    // Mark rectangle as correct (green)
                    Imgproc.rectangle(img_show, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y + roi.height), new Scalar(0, 255, 0), 2);
                    Log.i("No fault detected in Rectangle:", String.valueOf(n + 1));
                    // Add this rectangle to correctRectangles
                    correctRectangles.add(n);
                }
            } else {
                Log.i("CAD", "No contours with non-zero area found in Rectangle");
            }
        }

        // Handle overlapping rectangles
        for (int i = 0; i < a.size(); i++) {
            for (int j = i + 1; j < a.size(); j++) {
                RectF rectFi = a.get(i);
                RectF rectFj = a.get(j);
                int[] rect1 = {(int) rectFi.left, (int) rectFi.top, (int) rectFi.right, (int) rectFi.bottom};
                int[] rect2 = {(int) rectFj.left, (int) rectFj.top, (int) rectFj.right, (int) rectFj.bottom};

                double intersection = utils.intersectionArea(rect1, rect2);
                double rect1Area = (rectFi.right - rectFi.left) * (rectFi.bottom - rectFi.top);
                double rect2Area = (rectFj.right - rectFj.left) * (rectFj.bottom - rectFj.top);
                double overlapPercentage1 = (intersection / rect1Area) * 100;
                double overlapPercentage2 = (intersection / rect2Area) * 100;
                Log.i("overlapPercentage1:", String.valueOf(overlapPercentage1));
                Log.i("overlapPercentage2:", String.valueOf(overlapPercentage2));
                if (overlapPercentage1 > 65 || overlapPercentage2 > 65 && bypass !=1 && overlapPercentage1 < 80 && overlapPercentage2 < 80) {
                    if (overlapPercentage1 > overlapPercentage2) {
                        if (correctRectangles.contains(i)) {
                            correctRectangles.remove(Integer.valueOf(i));
                            faultyRectangles.add(i);
                            Rect roi = new Rect((int) rectFi.left, (int) rectFi.top, (int) (rectFi.right - rectFi.left), (int) (rectFi.bottom - rectFi.top));
                            Imgproc.rectangle(img_show, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y + roi.height), new Scalar(255, 0, 0), 2);
                            Log.i("Rectangle marked as faulty due to overlap with another rectangle:", String.valueOf(i + 1));
                        }
                    } else {
                        if (correctRectangles.contains(j)) {
                            correctRectangles.remove(Integer.valueOf(j));
                            faultyRectangles.add(j);
                            Rect roi = new Rect((int) rectFj.left, (int) rectFj.top, (int) (rectFj.right - rectFj.left), (int) (rectFj.bottom - rectFj.top));
                            Imgproc.rectangle(img_show, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y + roi.height), new Scalar(255, 0, 0), 2);
                            Log.i("Rectangle marked as faulty due to overlap with another rectangle:", String.valueOf(j + 1));
                        }
                    }
                }
            }
        }

        List<Object> output = new ArrayList<>();
        output.add(correctRectangles.size());
        output.add(img_show);
        return output;
    }
}
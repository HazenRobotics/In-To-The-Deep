package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Specimen implements VisionProcessor {

    public enum PixelColor {
        GREEN, PURPLE, YELLOW, WHITE
    }

    PixelColor color = PixelColor.GREEN;
    public Scalar blueLowerBoundHSV = new Scalar( 111, 215, 41 );
    public Scalar blueUpperBoundHSV = new Scalar( 255, 255, 123 );
    public Scalar redLowerBoundHSV = new Scalar( 0, 220, 35 );
    public Scalar redUpperBoundHSV = new Scalar( 0, 255, 255 );

    Mat temp = new Mat( );
    Mat red = new Mat( );
    Mat blue = new Mat( );

    Mat kernel = Mat.ones( 3, 3, CvType.CV_32F );

    ArrayList<Rect> redRects = new ArrayList<>( );
    ArrayList<Rect> blueRects = new ArrayList<>( );


    @Override
    public void init( int width, int height, CameraCalibration calibration ) {
    }

    @Override
    public Object processFrame( Mat frame, long captureTimeNanos ) {
        Imgproc.cvtColor( frame, temp, Imgproc.COLOR_RGB2HSV );

        Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_ERODE, kernel, new Point( 0, 0 ), 3 );
        Imgproc.morphologyEx( temp, temp, Imgproc.MORPH_DILATE, kernel, new Point( 0, 0 ), 4 );

        Core.inRange( temp, redLowerBoundHSV, redUpperBoundHSV, red );
        Core.inRange( temp, blueLowerBoundHSV, blueUpperBoundHSV, blue );

        List<MatOfPoint> contours = new ArrayList<>( );

        findBoundingBoxes( red, redRects, contours );
        findBoundingBoxes( blue, blueRects, contours );

        return frame;
    }

    @Override
    public void onDrawFrame( Canvas canvas, int onscreenWidth, int onscreenHeight,
                             float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext ) {
        Paint paint = new Paint();
        paint.setColor( Color.GREEN );
        paint.setStyle( Paint.Style.STROKE );
        paint.setStrokeWidth( scaleCanvasDensity * 4 );
        int minSize = 2000;
        paint.setColor( Color.RED );

        drawBoundingBoxes( canvas, paint, scaleBmpPxToCanvasPx, redRects, minSize );

        paint.setColor( Color.BLUE );
        drawBoundingBoxes( canvas, paint, scaleBmpPxToCanvasPx, blueRects, minSize );



    }

    private android.graphics.Rect makeGraphicsRect( Rect rect, float scaleBmpPxToCanvasPx ) {
        int left = Math.round( rect.x * scaleBmpPxToCanvasPx );
        int top = Math.round( rect.y * scaleBmpPxToCanvasPx );
        int right = left + Math.round( rect.width * scaleBmpPxToCanvasPx );
        int bottom = top + Math.round( rect.height * scaleBmpPxToCanvasPx );

        return new android.graphics.Rect( left, top, right, bottom );
    }

    /**
     * Converts a masked matrix to contours and finds bounding boxes from contours, putting them in a provided array list
     * @param mat masked matrix
     * @param rects array list to store rectangles
     * @param contourList empty list to store contours
     */
    private void findBoundingBoxes( Mat mat, ArrayList<Rect> rects, List<MatOfPoint> contourList ) {
        Imgproc.findContours( mat, contourList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );

        rects.clear();
        for( int i = 0; i < contourList.size( ); i++ ) {
            MatOfPoint point = contourList.get( i );
            Rect boundingRect = Imgproc.boundingRect( point );
            rects.add( boundingRect );
        }
        contourList.clear();
    }

    private void drawBoundingBoxes( Canvas canvas, Paint paint, float scaleBmpPxToCanvasPx,
                                    ArrayList<Rect> rects, int minSize) {
        for( int i = 0; i < rects.size( ); i++ ) {
            if (rects.get(i) != null) {
                android.graphics.Rect rect = makeGraphicsRect( rects.get( i ), scaleBmpPxToCanvasPx );
                if (rect.width() * rect.height() > minSize)
                    canvas.drawRect( rect, paint );
            }
        }
    }



}

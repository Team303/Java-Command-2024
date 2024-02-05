package frc.robot.util;

import java.util.Arrays;
import java.util.List;

public class TrackedNote {
    private Point2d bottomRightCoord;
    private Point2d topLeftCoord;
    private double confidence;

    public TrackedNote(double bx, double by, double tx, double ty, double confidence) {
        this.bottomRightCoord = new Point2d(bx, by);
        this.topLeftCoord = new Point2d(tx, ty);
        this.confidence = confidence;

    }

    /**
     * 
     * @param values Length 5 array with [bx,by,tx,ty,confidence]
     */
    public TrackedNote(double[] values) {
        if (values.length != 5) {
            throw new IllegalArgumentException("Tracked notes must enter arrays of length 5.");
        }
        this.bottomRightCoord = new Point2d(values[0], values[1]);
        this.topLeftCoord = new Point2d(values[2], values[3]);
        this.confidence = values[4];
    }
    /**
     * 
     * @return [bottom rights, top left]
     */
    public List<Point2d> getRectCorners(){
        return Arrays.asList(bottomRightCoord,topLeftCoord);
    }
    public double getConfidence(){
        return this.confidence;
    }
}

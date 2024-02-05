
package frc.modules;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.regex.Pattern;

import org.opencv.core.Point;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.targeting.PNPResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LowestAmbiguityComparator;
import frc.robot.util.Point2d;
import frc.robot.util.TrackedNote;

import static frc.robot.RobotMap.PhotonvisionConstants.CAMERA_MATRIX;
import static frc.robot.RobotMap.PhotonvisionConstants.DIST_COEFFS;

public class DetectorModule extends SubsystemBase {

    public static final ShuffleboardTab DETECTOR_TAB = Shuffleboard.getTab("Detector");

    public static final NetworkTable detector = NetworkTableInstance.getDefault().getTable("Detector");

    public static final GenericEntry ENTRIES = DETECTOR_TAB.add("Entries", "").getEntry();

    private List<TrackedNote> entries2D = new ArrayList<>();
    private List<PNPResult> entries3D = new ArrayList<>();

    public List<TrackedNote> getLatestResult2D() {
        return entries2D;
    }
    
    public List<PNPResult> getLatestResult3D() {
        return entries3D;
    }

    public boolean hasTargets() {
        return entries2D.size()>0;
    }


    public PNPResult getBestTarget() {
        if (hasTargets()) {
            return entries3D.get(0);
        }
        return null;
    }


    public Transform3d getPosition() {
        return getBestTarget().best;
    }

    /**
     * Assuming that only notes are being detected
     * TODO: Calibrate PiCam, also no idea if this works
     */
    public List<PNPResult> solvePNP(List<TrackedNote> boundingBoxes){
        List<PNPResult> results = new ArrayList<>();
        for(TrackedNote box : boundingBoxes){
            List<Point2d> points=box.getRectCorners();
            Point2d tl=points.get(1);
            Point2d br=points.get(0);
            Point[] imagePoints = {new Point(tl.x,tl.y),new Point(br.x,tl.y),new Point(br.x,br.y),new Point(tl.x,br.y)};
            List<Translation3d>  modelTrls = Arrays.asList(new Translation3d(0,-0.1778,0.1778),new Translation3d(0,0.1778,0.1778),new Translation3d(0,0.1778,-0.1778),new Translation3d(0,-0.1778,-0.1778));
            results.add(OpenCVHelp.solvePNP_SQPNP(CAMERA_MATRIX, DIST_COEFFS, modelTrls, imagePoints));
        }
        Collections.sort(results,new LowestAmbiguityComparator());
        return results;
    }


    @Override
    public void periodic() {
        entries2D.clear();
        String[] rawEntries = ENTRIES.getStringArray(null);
        if(rawEntries!=null){
            for(String entry : rawEntries) {
                //no idea if this is correct
                Pattern pattern = Pattern.compile(" |,");
                double[] formatted = (pattern.splitAsStream(entry).mapToDouble(Double::parseDouble).toArray());
                entries2D.add(new TrackedNote(formatted));
            }
        }
        entries3D = solvePNP(entries2D);
    }
}

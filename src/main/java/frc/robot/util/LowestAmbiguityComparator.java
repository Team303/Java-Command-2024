package frc.robot.util;

import java.util.Comparator;

import org.photonvision.targeting.PNPResult;

public class LowestAmbiguityComparator implements Comparator<PNPResult> {
    @Override
    public int compare(PNPResult one, PNPResult two){
        return Double.compare(one.ambiguity,two.ambiguity);
    }
}

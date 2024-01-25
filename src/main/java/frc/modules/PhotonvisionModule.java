package frc.modules;
// import static frc.robot.Robot.ALLIANCE_SUBSTATION_ID;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import frc.robot.RobotMap.PhotonvisionConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonvisionModule extends SubsystemBase {

    public static final ShuffleboardTab PHOTONVISION_TAB = Shuffleboard.getTab("PhotonVision");

    public static final NetworkTable photonvision = NetworkTableInstance.getDefault().getTable("PhotonVision");

    public static final GenericEntry FRONT_APRILTAG_ID = PHOTONVISION_TAB.add("Front April ID", 0).getEntry();
    public static final GenericEntry FRONT_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("Front ID Ambiguity", 0).getEntry();
    public static final GenericEntry FRONT_TARGET_YAW = PHOTONVISION_TAB.add("Front ID Yaw", 0).getEntry();
    public static final GenericEntry FRONT_TARGET_PITCH = PHOTONVISION_TAB.add("Front ID Pitch", 0).getEntry();
    public static final GenericEntry FRONT_TARGET_SKEW = PHOTONVISION_TAB.add("Front ID Skew", 0).getEntry();

    public static final GenericEntry BACK_APRILTAG_ID = PHOTONVISION_TAB.add("BACK April ID", 0).getEntry();
    public static final GenericEntry BACK_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("BACK ID Ambiguity", 0).getEntry();
    public static final GenericEntry BACK_TARGET_YAW = PHOTONVISION_TAB.add("BACK ID Yaw", 0).getEntry();
    public static final GenericEntry BACK_TARGET_PITCH = PHOTONVISION_TAB.add("BACK ID Pitch", 0).getEntry();
    public static final GenericEntry BACK_TARGET_SKEW = PHOTONVISION_TAB.add("BACK ID Skew", 0).getEntry();

    public static final GenericEntry LEFT_APRILTAG_ID = PHOTONVISION_TAB.add("LEFT April ID", 0).getEntry();
    public static final GenericEntry LEFT_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("LEFT ID Ambiguity", 0).getEntry();
    public static final GenericEntry LEFT_TARGET_YAW = PHOTONVISION_TAB.add("LEFT ID Yaw", 0).getEntry();
    public static final GenericEntry LEFT_TARGET_PITCH = PHOTONVISION_TAB.add("LEFT ID Pitch", 0).getEntry();
    public static final GenericEntry LEFT_TARGET_SKEW = PHOTONVISION_TAB.add("LEFT ID Skew", 0).getEntry();

    public static final GenericEntry RIGHT_APRILTAG_ID = PHOTONVISION_TAB.add("RIGHT April ID", 0).getEntry();
    public static final GenericEntry RIGHT_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("RIGHT ID Ambiguity", 0).getEntry();
    public static final GenericEntry RIGHT_TARGET_YAW = PHOTONVISION_TAB.add("RIGHT ID Yaw", 0).getEntry();
    public static final GenericEntry RIGHT_TARGET_PITCH = PHOTONVISION_TAB.add("RIGHT ID Pitch", 0).getEntry();
    public static final GenericEntry RIGHT_TARGET_SKEW = PHOTONVISION_TAB.add("RIGHT ID Skew", 0).getEntry();


    private static PhotonCamera[] camera = 
     {
           new PhotonCamera("PhotonVision3"),
        //    new PhotonCamera("PhotonVision2"),
            // new PhotonCamera("apple"),
            new PhotonCamera("PhotonVision1")
    };

    // public static enum PhotonPipeline {
    // AprilTag,
    // VCone,
    // CubePipline,
    // }
    

    public static enum CameraName {
        CAM3,
        // CAM2,
        // CAM4,
        CAM1
    }

    // public static enum ConePosition {
    // Up,
    // Down
    // }

    public PhotonCamera getCamera(CameraName name) {
        return camera[name.ordinal()];
    }

    public PhotonPipelineResult getLatestResult(CameraName name) {
        return camera[name.ordinal()].getLatestResult();
    }

    public Boolean hasTargets(CameraName name) {
        return getLatestResult(name).hasTargets();
    }

    public List<PhotonTrackedTarget> getTargetList(CameraName name) {
        return getLatestResult(name).getTargets();
    }

    public PhotonTrackedTarget getBestTarget(CameraName name) {
        if (hasTargets(name)) {
            return getLatestResult(name).getBestTarget();
        }
        return null;
    }

    public List<TargetCorner> getRectCorners(CameraName name) {
        return getBestTarget(name).getMinAreaRectCorners();
    }

    public List<TargetCorner> getCorners(CameraName name) {
        return getBestTarget(name).getDetectedCorners();
    }

    public Transform3d getPosition(CameraName name) {
        return getBestTarget(name).getBestCameraToTarget();
    }

    public void takeImage(CameraName name) {
        getCamera(name).takeInputSnapshot();
    }

    public void getImages(CameraName name) {
        getCamera(name).takeOutputSnapshot();
    }
    // public void setPipeline(CameraName name, PhotonPipeline pipelineName) {
    // getCamera(name).setPipelineIndex(pipelineName.ordinal());
    // }

    // public PhotonPipeline getPipeline(CameraName name) {
    // return PhotonPipeline.values()[getCamera(name).getPipelineIndex()];
    // }

    // public ConePosition getConePosition(CameraName name) {
    // double skew = getObjectSkew(name);
    // if (skew % 360 <= 5) {
    // return ConePosition.Up;
    // } else {
    // return ConePosition.Down;
    // }
    // }

    public double getObjectSkew(CameraName name) {
        PhotonTrackedTarget target = getBestTarget(name);
        double skew = target.getSkew();

        return skew;
    }

    public double getObjectYaw(CameraName name) {
        PhotonTrackedTarget target = getBestTarget(name);
        double yaw = target.getYaw();

        return yaw;
    }

    // //TODO: Make it work for 2024 and for all cameras
    // public double getDistanceToTarget(CameraName camera) {
    // if (!hasTargets(camera)) {
    // return Double.NaN;
    // }

    // int id = getBestTarget(camera).getFiducialId();
    // if (id != ALLIANCE_SUBSTATION_ID) {
    // return PhotonUtils.calculateDistanceToTargetMeters(
    // PhotonvisionConstants.FRONT_CAMERA_HEIGHT_METERS,
    // PhotonvisionConstants.GRID_TARGET_HEIGHT_METERS,
    // PhotonvisionConstants.CAMERA_PITCH_RADIANS,
    // Units.degreesToRadians(getBestTarget(camera).getPitch()));
    // } else {
    // return PhotonUtils.calculateDistanceToTargetMeters(
    // PhotonvisionConstants.FRONT_CAMERA_HEIGHT_METERS,
    // PhotonvisionConstants.DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS,
    // PhotonvisionConstants.CAMERA_PITCH_RADIANS,
    // Units.degreesToRadians(getBestTarget(camera).getPitch()));
    // }
    // }

    @Override
    public void periodic() {

        PhotonTrackedTarget targetFront = getBestTarget(CameraName.CAM1);
        // PhotonTrackedTarget targetRight = getBestTarget(CameraName.CAM2);
        PhotonTrackedTarget targetBack = getBestTarget(CameraName.CAM3);
        // PhotonTrackedTarget targetLeft = getBestTarget(CameraName.CAM4);

        if (targetFront != null) {
            // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
            FRONT_APRILTAG_ID.setInteger(targetFront.getFiducialId());
            // }

            FRONT_TARGET_AMBIGUITY.setDouble(targetFront.getPoseAmbiguity());
            FRONT_TARGET_YAW.setDouble(targetFront.getYaw());
            FRONT_TARGET_PITCH.setDouble(targetFront.getPitch());
            FRONT_TARGET_SKEW.setDouble(targetFront.getSkew());
        }
        if (targetBack != null) {
            // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
            BACK_APRILTAG_ID.setInteger(targetBack.getFiducialId());
            // }

            BACK_TARGET_AMBIGUITY.setDouble(targetBack.getPoseAmbiguity());
            BACK_TARGET_YAW.setDouble(targetBack.getYaw());
            BACK_TARGET_PITCH.setDouble(targetBack.getPitch());
            BACK_TARGET_SKEW.setDouble(targetBack.getSkew());
        }
        // if (targetLeft != null) {

        //     // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
        //     LEFT_APRILTAG_ID.setInteger(targetLeft.getFiducialId());
        //     // }

        //     LEFT_TARGET_AMBIGUITY.setDouble(targetLeft.getPoseAmbiguity());
        //     LEFT_TARGET_YAW.setDouble(targetLeft.getYaw());
        //     LEFT_TARGET_PITCH.setDouble(targetLeft.getPitch());
        //     LEFT_TARGET_SKEW.setDouble(targetLeft.getSkew());
        // }
        // if (targetRight != null) {

        //     // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
        //     RIGHT_APRILTAG_ID.setInteger(targetRight.getFiducialId());
        //     // }

        //     RIGHT_TARGET_AMBIGUITY.setDouble(targetRight.getPoseAmbiguity());
        //     RIGHT_TARGET_YAW.setDouble(targetRight.getYaw());
        //     RIGHT_TARGET_PITCH.setDouble(targetRight.getPitch());
        //     RIGHT_TARGET_SKEW.setDouble(targetRight.getSkew());
        // }
    }
}
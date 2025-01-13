package frc.modules;
// import static frc.robot.Robot.ALLIANCE_SUBSTATION_ID;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonvisionModule extends SubsystemBase {

    public static final ShuffleboardTab PHOTONVISION_TAB = Shuffleboard.getTab("PhotonVision");

    public static final NetworkTable photonvision = NetworkTableInstance.getDefault().getTable("PhotonVision");

    public static final GenericEntry FRONT_LEFT_APRILTAG_ID = PHOTONVISION_TAB.add("Front Left April ID", 0).getEntry();
    public static final GenericEntry FRONT_LEFT_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("Front Left ID Ambiguity", 0).getEntry();
    public static final GenericEntry FRONT_LEFT_TARGET_YAW = PHOTONVISION_TAB.add("Front Left ID Yaw", 0).getEntry();
    public static final GenericEntry FRONT_LEFT_TARGET_PITCH = PHOTONVISION_TAB.add("Front Left ID Pitch", 0).getEntry();
    public static final GenericEntry FRONT_LEFT_TARGET_SKEW = PHOTONVISION_TAB.add("Front Left ID Skew", 0).getEntry();

    public static final GenericEntry FRONT_RIGHT_APRILTAG_ID = PHOTONVISION_TAB.add("Front Right April ID", 0).getEntry();
    public static final GenericEntry FRONT_RIGHT_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("Front Right ID Ambiguity", 0).getEntry();
    public static final GenericEntry FRONT_RIGHT_TARGET_YAW = PHOTONVISION_TAB.add("Front Right ID Yaw", 0).getEntry();
    public static final GenericEntry FRONT_RIGHT_TARGET_PITCH = PHOTONVISION_TAB.add("Front Right ID Pitch", 0).getEntry();
    public static final GenericEntry FRONT_RIGHT_TARGET_SKEW = PHOTONVISION_TAB.add("Front Right ID Skew", 0).getEntry();

    // public static final GenericEntry LEFT_APRILTAG_ID = PHOTONVISION_TAB.add("LEFT April ID", 0).getEntry();
    // public static final GenericEntry LEFT_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("LEFT ID Ambiguity", 0).getEntry();
    // public static final GenericEntry LEFT_TARGET_YAW = PHOTONVISION_TAB.add("LEFT ID Yaw", 0).getEntry();
    // public static final GenericEntry LEFT_TARGET_PITCH = PHOTONVISION_TAB.add("LEFT ID Pitch", 0).getEntry();
    // public static final GenericEntry LEFT_TARGET_SKEW = PHOTONVISION_TAB.add("LEFT ID Skew", 0).getEntry();

    // public static final GenericEntry RIGHT_APRILTAG_ID = PHOTONVISION_TAB.add("RIGHT April ID", 0).getEntry();
    // public static final GenericEntry RIGHT_TARGET_AMBIGUITY = PHOTONVISION_TAB.add("RIGHT ID Ambiguity", 0).getEntry();
    // public static final GenericEntry RIGHT_TARGET_YAW = PHOTONVISION_TAB.add("RIGHT ID Yaw", 0).getEntry();
    // public static final GenericEntry RIGHT_TARGET_PITCH = PHOTONVISION_TAB.add("RIGHT ID Pitch", 0).getEntry();
    // public static final GenericEntry RIGHT_TARGET_SKEW = PHOTONVISION_TAB.add("RIGHT ID Skew", 0).getEntry();

    private PhotonCamera[] camera;

    public PhotonvisionModule() {
        PhotonCamera[] photonCamera = { new PhotonCamera("PhotonVision4"),
                // new PhotonCamera("PhotonVision2"),
                // new PhotonCamera("apple"),
                new PhotonCamera("PhotonVision2") };
        camera = photonCamera;
    }

    // public static enum PhotonPipeline {
    // AprilTag,
    // VCone,
    // CubePipline,
    // }

    public static enum CameraName {
        FRONT_LEFT,
        // CAM2,
        // CAM4,
        FRONT_RIGHT
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

        PhotonTrackedTarget targetFrontRight = getBestTarget(CameraName.FRONT_RIGHT);
        // PhotonTrackedTarget targetRight = getBestTarget(CameraName.CAM2);
        PhotonTrackedTarget targetFrontLeft = getBestTarget(CameraName.FRONT_LEFT);
        // PhotonTrackedTarget targetLeft = getBestTarget(CameraName.CAM4);

        if (targetFrontLeft != null) {
            // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
            FRONT_LEFT_APRILTAG_ID.setInteger(targetFrontLeft.getFiducialId());
            // }

            FRONT_LEFT_TARGET_AMBIGUITY.setDouble(targetFrontLeft.getPoseAmbiguity());
            FRONT_LEFT_TARGET_YAW.setDouble(targetFrontLeft.getYaw());
            FRONT_LEFT_TARGET_PITCH.setDouble(targetFrontLeft.getPitch());
            FRONT_LEFT_TARGET_SKEW.setDouble(targetFrontLeft.getSkew());
        }
        if (targetFrontRight != null) {
            // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
            FRONT_RIGHT_APRILTAG_ID.setInteger(targetFrontRight.getFiducialId());
            // }

            FRONT_RIGHT_TARGET_AMBIGUITY.setDouble(targetFrontRight.getPoseAmbiguity());
            FRONT_RIGHT_TARGET_YAW.setDouble(targetFrontRight.getYaw());
            FRONT_RIGHT_TARGET_PITCH.setDouble(targetFrontRight.getPitch());
            FRONT_RIGHT_TARGET_SKEW.setDouble(targetFrontRight.getSkew());
        }
        // if (targetLeft != null) {

        // // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
        // LEFT_APRILTAG_ID.setInteger(targetLeft.getFiducialId());
        // // }

        // LEFT_TARGET_AMBIGUITY.setDouble(targetLeft.getPoseAmbiguity());
        // LEFT_TARGET_YAW.setDouble(targetLeft.getYaw());
        // LEFT_TARGET_PITCH.setDouble(targetLeft.getPitch());
        // LEFT_TARGET_SKEW.setDouble(targetLeft.getSkew());
        // }
        // if (targetRight != null) {

        // // if (getPipeline(CameraName.CAM1) == PhotonPipeline.AprilTag) {
        // RIGHT_APRILTAG_ID.setInteger(targetRight.getFiducialId());
        // // }

        // RIGHT_TARGET_AMBIGUITY.setDouble(targetRight.getPoseAmbiguity());
        // RIGHT_TARGET_YAW.setDouble(targetRight.getYaw());
        // RIGHT_TARGET_PITCH.setDouble(targetRight.getPitch());
        // RIGHT_TARGET_SKEW.setDouble(targetRight.getSkew());
        // }
    }
}
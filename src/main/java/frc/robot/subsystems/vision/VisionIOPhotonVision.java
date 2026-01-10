// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.FieldConstants;

/** Add your docs here. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult result = new PhotonPipelineResult();
    private Pose3d latestPose = new Pose3d();
    public VisionIOPhotonVision(String name, Transform3d transform) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(VisionConstants.PhysicalConstants.fieldLayout, VisionConstants.Strategies.primary, transform);
        poseEstimator.setMultiTagFallbackStrategy(VisionConstants.Strategies.secondary);
    }
    @Override
    public Pose3d getPose() {
        return latestPose;
    }

    @Override
    public void updatePose(Pose2d robotPose) {
        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), robotPose.getRotation());
    }
    
    @Override
    public void updateInputs(VisionInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.size() > 0) {
            result = results.get(0);
            poseEstimator.update(result).ifPresent((poseEstimated) -> {
                if (FieldConstants.inFieldBounds(poseEstimated.estimatedPose.toPose2d()) && MathUtil.applyDeadband(poseEstimated.estimatedPose.getZ(),0.1) == 0.0) {
                    latestPose = poseEstimated.estimatedPose;
                    inputs.estimatedPose = poseEstimated.estimatedPose;
                    inputs.pose = poseEstimated.estimatedPose.toPose2d();
                    inputs.tagCount = poseEstimated.targetsUsed.size();
                    Pose2d[] tagPoses = new Pose2d[inputs.tagCount];
                    for (int i=0; i<inputs.tagCount; i++) {
                        if (poseEstimated.targetsUsed.get(i).getFiducialId() != -1) {
                            tagPoses[i] = VisionConstants.PhysicalConstants.fieldLayout.getTagPose(poseEstimated.targetsUsed.get(i).getFiducialId()).get().toPose2d();
                        } else {
                            tagPoses[i] = new Pose2d();
                        }
                    }
                    inputs.timestamp = poseEstimated.timestampSeconds;
                }
            });
            inputs.type = result.getMultiTagResult().isPresent() ? ObservationType.PhotonMultiTag : (VisionConstants.Strategies.secondary == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE ? ObservationType.PhotonTrig : ObservationType.PhotonPnP);
            inputs.averageDistance = getAverageDist(result);
        }
    }

    private double getAverageDist(PhotonPipelineResult result) {
        if (result.hasTargets()) {
            double dist = result.getTargets().get(0).getBestCameraToTarget().getTranslation().getNorm();
            for (int i=1; i<result.getTargets().size(); i++) {
                dist += result.getTargets().get(i).getBestCameraToTarget().getTranslation().getNorm();
                dist /= 2;
            }
            return dist;
        } else {
            return 0.0;
        }
    }
}

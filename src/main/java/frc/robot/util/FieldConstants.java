// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class FieldConstants {
    public static AprilTagFieldLayout layout; // Use This when Field Gets Release
    static {
        try {
            layout = AprilTagFieldLayout.loadFromResource(Filesystem.getDeployDirectory()+"/fields/rebuilt.json");
        } catch (IOException e) {
            System.out.println("Failed to load custom field defaulting to WPILib default");
            if (layout == null) {
                layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            }
        }
    }

    public static boolean inFieldBounds(Pose2d pose) {
        return pose.getX() >= 0 && pose.getX() <= FieldConstants.layout.getFieldLength() && pose.getY() >= 0 && pose.getY() <= FieldConstants.layout.getFieldWidth();
    }
}

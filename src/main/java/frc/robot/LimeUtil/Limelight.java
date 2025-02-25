// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LimeUtil;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimeUtil.LimelightHelpers.LimelightResults;
import frc.robot.LimeUtil.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class Limelight 
{
    private final String limelightName;
    private int mostRecentTagIdSeen = 0;

    /**
     * 4 LED modes of a Limelight.
     * 
     * @param def Pipeline Default.
     * @param on Force On.
     * @param off Force Off.
     * @param blink Force Blink.
     */
    public enum LEDMode 
    {
        def, on, off, blink
    }

    /**
     *  3 different types of streams for use with USB Camera.
     * 
     * @param sideBySide A side-by-side stream of Limelight and Camera.
     * @param limelightAsPrimary A large view of the Limelight stream, with the Camera stream in the corner.
     * @param cameraAsPrimary A large view of the Camera stream, with the Limelight stream in the corner.
     */
    public enum StreamMode
    {
        sideBySide, limelightAsPrimary, cameraAsPrimary
    }

    /**
     * Creates a Limelight object.
     * 
     * @param limelightName The input name of a limelight. Default is 'limelight'.
     */
    public Limelight (String limelightName)
    {
        this.limelightName = limelightName;
        
    }

    /**
     * Sets the camera pose in the robot space (0, 0, 0 is dead center of the robot)
     * 
     * @param forwardMeters Limelight meters forward
     * @param rightMeters Limelight meters right
     * @param upMeters Limelight meters up
     * @param roll Limelight degrees in roll axis (-180 to 180)
     * @param pitch Limelight degrees in pitch axis (-180 to 180)
     * @param yaw Limelight degrees in yaw axis (-180 to 180)
     */
    public void setLimelightPosition(double forwardMeters, double rightMeters, double upMeters, double roll, double pitch, double yaw)
    {
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, forwardMeters, rightMeters, upMeters, roll, pitch, yaw);
    }

    /**
     * Sets the Limelight's current pipeline to the desired pipeline.
     * 
     * @param pipelineIndex Index of the desired pipeline.
     */
    public void setPipeline(int pipelineIndex)
    {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    /**
     * Overrides the current set LED mode of the limelight.
     * 
     * @param ledMode The desired {@link LEDMode}.
     */
    public void setLEDMode(LEDMode ledMode)
    {
        switch(ledMode)
        {
            case def:
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
                break;
            case on:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
            case off:
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
                break;
            case blink:
                LimelightHelpers.setLEDMode_ForceBlink(limelightName);
                break;
        }
    }

    /**
     * Sets the desired stream mode. For use with a USB Camera
     * 
     * @param streamMode The desired {@link StreamMode}.
     */
    public void setStreamMode(StreamMode streamMode)
    {
        switch(streamMode)
        {
            case sideBySide:
                LimelightHelpers.setStreamMode_Standard(limelightName);
                break;
            case limelightAsPrimary:
                LimelightHelpers.setStreamMode_PiPMain(limelightName);
                break;
            case cameraAsPrimary:
                LimelightHelpers.setStreamMode_PiPSecondary(limelightName);
                break;
        }
    }

    /**
     * @return The latest {@link LimelightResults} JSON Object
     */
    public LimelightResults getRawResults()
    {
        return LimelightHelpers.getLatestResults(limelightName);
    }

    /**
     * Takes a snapshot to view it later in the web interface
     * 
     * @param snapshotName Desired name of snapshot
     */
    public void takeSnapshot(String snapshotName)
    {
        LimelightHelpers.takeSnapshot(limelightName, snapshotName);
    }

    /**
     * @return Current estimated Pose2d.
     */
    public Pose2d getBotPose2D()
    {
        return LimelightHelpers.getBotPose2d(limelightName);
    }

    /**
     * @return Current {@link PoseEstimate} using wpiBlue coordinates.
     */
    public PoseEstimate getLimelightPoseEstimate_wpiBlue() 
    {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    /**
     * @return Current {@link PoseEstimate} using wpiRed coordinates.
     */
    public PoseEstimate getLimelightPoseEstimate_wpiRed() 
    {
        return LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
    }

    /**
     * Returns current MegaTag2 {@link PoseEstimate} using wpiBlue coordinates and gyro input.
     * MUCH MORE RELIABLE!
     * 
     * @param gyroYaw Current yaw of the gyro.
     * @return Current {@link PoseEstimate} using wpiBlue coordinates.
     */
    public PoseEstimate getLimelightPoseEstimate_wpiBlue_MegaTag2(double gyroYaw)
    {
        LimelightHelpers.SetRobotOrientation(limelightName, gyroYaw, 0, 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    /**
     * Returns current MegaTag2 {@link PoseEstimate} using wpiRed coordinates and gyro input.
     * MUCH MORE RELIABLE!
     * 
     * @param gyroYaw Current yaw of the gyro.
     * @return Current {@link PoseEstimate} using wpiRed coordinates.
     */
    public PoseEstimate getLimelightPoseEstimate_wpiRed_MegaTag2(double gyroYaw)
    {
        LimelightHelpers.SetRobotOrientation(limelightName, gyroYaw, 0, 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    }

    public int getMostRecentTagIdSeen() {
        return this.mostRecentTagIdSeen;
    }

    public void clearMostRecentTagIdSeen() {
        this.mostRecentTagIdSeen = 0;
    }

    public boolean tagIsSeen() {
        return LimelightHelpers.getTV(this.limelightName);
    }

    public int getPrimaryAprilTagID() {
        return (int)LimelightHelpers.getTID(this.limelightName);
    }

    public double getX() {
        return LimelightHelpers.getTX(this.limelightName);
    }

    public double getY() {
        return LimelightHelpers.getTX(this.limelightName);
    }
}

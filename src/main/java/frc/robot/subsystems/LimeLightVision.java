package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import java.util.List;

import edu.wpi.first.math.VecBuilder;

public class LimeLightVision
{
    private List<String> cameraNames = List.of("limelight-front");

    public LimeLightVision(List<String> cameraNames)
    {
        this.cameraNames = cameraNames;
    }

    /**
     * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses. Don't
     * call if rate of rotation is greater than 720 degrees per second!
     *
     * @param swerveDrive
     *            {@link SwerveDrive} instance.
     */
    public void updatePoseEstimation(CommandSwerveDrivetrain swerveDrive)
    {

        var state = swerveDrive.getState();
        var currentPose = state.Pose;
        var currentRotation = currentPose.getRotation().getDegrees();
        double yawRate = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);

        var pigeon = swerveDrive.getPigeon2();
        var pitch = pigeon.getPitch().getValueAsDouble();
        var roll = pigeon.getRoll().getValueAsDouble();

        cameraNames.forEach(
            cameraName ->
            {
                // setting pitch and roll will should help when robot leaves ground
                LimelightHelpers.SetRobotOrientation(
                    cameraName,
                    currentRotation,
                    yawRate,
                    pitch,
                    0,
                    roll,
                    0);

                // alway blue origin
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

                if (mt2 == null)
                {
                    return;
                }

                if (mt2.tagCount == 0)
                {
                    return;
                }

                double stdDev = 1.5; // Start with your 0.5 base
                if (mt2.tagCount == 1)
                {
                    stdDev = 1.0;
                }

                if (mt2.avgTagDist > 4.0)
                { // e.g., further than 4 meters
                    stdDev = stdDev * (1.0 + (mt2.avgTagDist - 4.0)); // Scales linearly
                }

                // set the ceiling for the vision stdDev to 3.0
                stdDev = Math.min(stdDev, 3.0);

                swerveDrive.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds,
                    VecBuilder.fill(stdDev, stdDev, 0.9));
            });
    }

    /* Set Throttle for all Limelight cameras */
    public void setCamThrottle(double throttle)
    {
        cameraNames.forEach(cameraName -> LimelightHelpers.SetThrottle(cameraName, (int) throttle));
    }
}

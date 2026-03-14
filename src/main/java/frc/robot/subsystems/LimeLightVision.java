package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import java.util.List;

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
        var currentRotation = currentPose.getRotation();
        double yawRate = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);

        cameraNames.forEach(
            cameraName ->
            {
                LimelightHelpers.SetRobotOrientation(
                    cameraName,
                    currentRotation.getDegrees(),
                    yawRate,
                    0,
                    0,
                    0,
                    0);

                LimelightHelpers.PoseEstimate mt2;

                // alway blue origin
                mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

                if (mt2 == null)
                {
                    return;
                }

                if (mt2.tagCount == 0)
                {
                    return;
                }

                swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            });
    }

    /* Set Throttle for all Limelight cameras */
    public void setCamThrottle(double throttle)
    {
        cameraNames.forEach(cameraName -> LimelightHelpers.SetThrottle(cameraName, (int) throttle));
    }
}

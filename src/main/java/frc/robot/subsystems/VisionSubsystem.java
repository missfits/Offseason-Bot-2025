package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import frc.robot.Constants.VisionConstants;


public class VisionSubsystem extends SubsystemBase{
    private ArrayList<LocalizationCamera> cameras = new ArrayList<>();
    private List<LocalizationCamera> camerasWithValidPose = new ArrayList<>();
  
    /** Creates a new Vision Subsystem. */
    public VisionSubsystem() {
      cameras.add(new LocalizationCamera(VisionConstants.CAMERA1_NAME, VisionConstants.ROBOT_TO_CAM1_3D));
      cameras.add(new LocalizationCamera(VisionConstants.CAMERA2_NAME, VisionConstants.ROBOT_TO_CAM2_3D));
    }
  
    public List<LocalizationCamera> getLocalizationCameras(){
      return camerasWithValidPose;
    }
  
    @Override
    public void periodic() {
      for (LocalizationCamera cam : cameras){
        cam.findTarget();
      }

      // sorts the camera readings by time (care less about older readings)
      camerasWithValidPose = cameras.stream() // turn the list into a stream
      .filter((camera) -> { // only get the cameras with a valid EstimatedRobotPose
           return camera.getRobotPose() != null && camera.getTargetFound();
      })
      .sorted((camera_a, camera_b) -> { // simplified comparator because we've filtered out invalid readings.
           EstimatedRobotPose poseA = camera_a.getRobotPose();
           EstimatedRobotPose poseB = camera_b.getRobotPose();
           return Double.compare(poseA.timestampSeconds, poseB.timestampSeconds);
       })
      .toList();
      }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
}

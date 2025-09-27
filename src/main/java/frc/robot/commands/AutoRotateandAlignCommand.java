package frc.robot.commands;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionUtils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class AutoRotateandAlignCommand extends Command {
    public enum ReefPosition {
        LEFT, RIGHT, CENTER
    }
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private final ReefPosition m_side;
  private boolean reachedIntermediateTranslation = false; 

  private Rotation2d m_targetRotation;
  private Translation2d m_targetTranslation;
  private Translation2d m_targetIntermediateTranslation;

  
  private final TrapezoidProfile.Constraints intermediateConstraints = new TrapezoidProfile.Constraints(AutoAlignConstants.kMaxIntermediateV, AutoAlignConstants.kMaxIntermediateA);
  private final TrapezoidProfile.Constraints normalConstraints = new TrapezoidProfile.Constraints(AutoAlignConstants.kMaxV, AutoAlignConstants.kMaxA);


  private final ProfiledPIDController xController = new ProfiledPIDController(DrivetrainConstants.AUTOALIGN_POSITION_P, DrivetrainConstants.AUTOALIGN_POSITION_I, DrivetrainConstants.AUTOALIGN_POSITION_D, new TrapezoidProfile.Constraints(AutoAlignConstants.kMaxV, AutoAlignConstants.kMaxA));
  private final ProfiledPIDController yController = new ProfiledPIDController(DrivetrainConstants.AUTOALIGN_POSITION_P, DrivetrainConstants.AUTOALIGN_POSITION_I, DrivetrainConstants.AUTOALIGN_POSITION_D, new TrapezoidProfile.Constraints(AutoAlignConstants.kMaxV, AutoAlignConstants.kMaxA));
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoRotateandAlignCommand(CommandSwerveDrivetrain drivetrain, ReefPosition side) {
    m_drivetrain = drivetrain;
    m_side = side;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // add visionutils later (assuming it exists)
    Pose2d aprilTagPose = VisionUtils.getClosestReefAprilTag(m_drivetrain.getState().Pose);

    if (aprilTagPose != null){
      // if we assume aligning to the center, want to be Robot size /2 away from the april tag at a perpendicular angle. 
      // then we align to the left/right/center
      // essentially calculating the targetPose based off of the apriltagPose
      if (m_side.equals(ReefPosition.RIGHT)){
        m_targetTranslation = new Translation2d(
            aprilTagPose.getX()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(aprilTagPose.getRotation().getRadians())
            + AutoAlignConstants.REEF_OFFSET_RIGHT * Math.cos(Math.PI/2 + aprilTagPose.getRotation().getRadians()), 
            aprilTagPose.getY()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(aprilTagPose.getRotation().getRadians())
            + AutoAlignConstants.REEF_OFFSET_RIGHT * Math.sin(Math.PI/2 + aprilTagPose.getRotation().getRadians()));
      }

      else if (m_side.equals(ReefPosition.LEFT)){
        m_targetTranslation = new Translation2d(
            aprilTagPose.getX()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(aprilTagPose.getRotation().getRadians())
            - AutoAlignConstants.REEF_OFFSET_LEFT * Math.cos(Math.PI/2 + aprilTagPose.getRotation().getRadians()), 
            aprilTagPose.getY()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(aprilTagPose.getRotation().getRadians())
            - AutoAlignConstants.REEF_OFFSET_LEFT * Math.sin(Math.PI/2 + aprilTagPose.getRotation().getRadians()));
      }
      else{
        m_targetTranslation = new Translation2d(
          aprilTagPose.getX() 
          + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(aprilTagPose.getRotation().getRadians()),
          aprilTagPose.getY()
          + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(aprilTagPose.getRotation().getRadians()));
    
      }
      m_targetIntermediateTranslation = m_targetTranslation.plus(new Translation2d(
            AutoAlignConstants.INTERMEDIATE_POS_DIST * Math.cos(aprilTagPose.getRotation().getRadians()),
            AutoAlignConstants.INTERMEDIATE_POS_DIST * Math.sin(aprilTagPose.getRotation().getRadians())
          ));
    }
    else{
      m_targetTranslation = m_drivetrain.getState().Pose.getTranslation();
      m_targetIntermediateTranslation = m_drivetrain.getState().Pose.getTranslation();
    }

    // getting the target rotation by taking the rotation of the apriltag but then adding pi since want to be perpendicular
    m_targetRotation = aprilTagPose.getRotation().plus(Rotation2d.fromRadians(Math.PI));


    xController.reset(m_drivetrain.getState().Pose.getX());
    yController.reset(m_drivetrain.getState().Pose.getY());

    driveRequest.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROTATION_kP, DrivetrainConstants.ROTATION_kP, DrivetrainConstants.ROTATION_kP);
    driveRequest.HeadingController.enableContinuousInput(0, Math.PI * 2);
      
    SmartDashboard.putString("drivetoreef/target robot rotation", m_targetRotation.toString());
    SmartDashboard.putString("drivetoreef/target robot translation", m_targetTranslation.toString());
    SmartDashboard.putNumber("drivetoreef/targetX", m_targetTranslation.getX());
    SmartDashboard.putNumber("drivetoreef/targetY", m_targetTranslation.getY());
    SmartDashboard.putNumber("drivetoreef/targetRotation", m_targetRotation.getRadians());
    SmartDashboard.putBoolean("drivetoreef/reachedIntermediateTranslation", reachedIntermediateTranslation);

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xVelocity;
    double yVelocity;

    // at intermediate translation, may want to go at a slower velocity/acceleration
    
    if (! reachedIntermediateTranslation) {
      xController.setConstraints(normalConstraints);
      yController.setConstraints(normalConstraints);
      
      // getting the x and y velocities and using the trapezoid constraints so that it updates velocity based on position
      xVelocity = xController.calculate(m_drivetrain.getState().Pose.getX(), m_targetIntermediateTranslation.getX()) + xController.getSetpoint().velocity;
      yVelocity = yController.calculate(m_drivetrain.getState().Pose.getY(), m_targetIntermediateTranslation.getY()) + yController.getSetpoint().velocity;
      // checking if aligned
      reachedIntermediateTranslation = isAligned(m_targetIntermediateTranslation);
    } 
    else {
      xController.setConstraints(intermediateConstraints);
      yController.setConstraints(intermediateConstraints);

      xVelocity = xController.calculate(m_drivetrain.getState().Pose.getX(), m_targetTranslation.getX()) + xController.getSetpoint().velocity;
      yVelocity = yController.calculate(m_drivetrain.getState().Pose.getY(), m_targetTranslation.getY()) + yController.getSetpoint().velocity;
    }
        
  
    m_drivetrain.setControl(driveRequest
    .withVelocityX(xVelocity)
    .withVelocityY(yVelocity)
    .withTargetDirection(m_targetRotation));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // checking if the drivetrain is aligned and discarding a small margin of error
  public boolean isAligned(Translation2d translation) {
    Pose2d drivetrainPose = m_drivetrain.getState().Pose;
    // change to absolute value?
    double xDist = Math.abs(drivetrainPose.getX() - translation.getX());
    double yDist = Math.abs(drivetrainPose.getY() - translation.getY());
    SmartDashboard.putNumber("drivetoreef/auto-alignment-xdist", xDist);
    SmartDashboard.putNumber("drivetoreef/auto-alignment-ydist", yDist);
    return ((xDist < VisionConstants.VISION_ALIGNMENT_DISCARD) && (yDist < VisionConstants.VISION_ALIGNMENT_DISCARD));
  }
}

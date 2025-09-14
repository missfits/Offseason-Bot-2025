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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class AutorotateCommand extends Command {
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
  public AutorotateCommand(CommandSwerveDrivetrain drivetrain, ReefPosition side) {
    m_drivetrain = drivetrain;
    m_side = side;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

    // add visionutils later (assuming it exists)
    Pose2d targetPose = VisionUtils.getClosestReefAprilTag(m_drivetrain.getState().Pose);


    

    if (targetPose != null){
      if (m_side.equals(ReefPosition.RIGHT)){
        m_targetTranslation = new Translation2d(
            targetPose.getX()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(targetPose.getRotation().getRadians())
            + AutoAlignConstants.REEF_OFFSET_RIGHT * Math.cos(Math.PI/2 + targetPose.getRotation().getRadians()), 
            targetPose.getY()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(targetPose.getRotation().getRadians())
            + AutoAlignConstants.REEF_OFFSET_RIGHT * Math.sin(Math.PI/2 + targetPose.getRotation().getRadians()));
      }

      else if (m_side.equals(ReefPosition.LEFT)){
        m_targetTranslation = new Translation2d(
            targetPose.getX()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(targetPose.getRotation().getRadians())
            - AutoAlignConstants.REEF_OFFSET_LEFT * Math.cos(Math.PI/2 + targetPose.getRotation().getRadians()), 
            targetPose.getY()
            + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(targetPose.getRotation().getRadians())
            - AutoAlignConstants.REEF_OFFSET_LEFT * Math.sin(Math.PI/2 + targetPose.getRotation().getRadians()));
      }
      else{
        m_targetTranslation = new Translation2d(
          targetPose.getX() 
          + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.cos(targetPose.getRotation().getRadians()),
          targetPose.getY()
          + DrivetrainConstants.ROBOT_SIZE_X/2 * Math.sin(targetPose.getRotation().getRadians()));
    
      }
      m_targetIntermediateTranslation = m_targetTranslation.plus(new Translation2d(
            AutoAlignConstants.INTERMEDIATE_POS_DIST * Math.cos(targetPose.getRotation().getRadians()),
            AutoAlignConstants.INTERMEDIATE_POS_DIST * Math.sin(targetPose.getRotation().getRadians())
          ));
    }
    m_targetRotation = targetPose.getRotation().plus(Rotation2d.fromRadians(Math.PI));

    xController.reset(m_drivetrain.getState().Pose.getX());
    yController.reset(m_drivetrain.getState().Pose.getY());

    driveRequest.HeadingController = new PhoenixPIDController(DrivetrainConstants.AUTOALIGN_POSITION_P, DrivetrainConstants.AUTOALIGN_POSITION_I, DrivetrainConstants.AUTOALIGN_POSITION_D);
    driveRequest.HeadingController.enableContinuousInput(0, Math.PI * 2);
      
    SmartDashboard.putString("drivetoreef/target robot rotation", m_targetRotation.toString());

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xVelocity;
    double yVelocity;

    
    if (! reachedIntermediateTranslation) {
      xController.setConstraints(normalConstraints);
      yController.setConstraints(normalConstraints);
    
      xVelocity = xController.calculate(m_drivetrain.getState().Pose.getX(), m_targetIntermediateTranslation.getX()) + xController.getSetpoint().velocity;
      yVelocity = yController.calculate(m_drivetrain.getState().Pose.getY(), m_targetIntermediateTranslation.getY()) + yController.getSetpoint().velocity;
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
  public boolean isAligned(Translation2d translation) {
    Pose2d drivetrainPose = m_drivetrain.getState().Pose;
    // change to absolute value?
    double xDist = Math.abs(drivetrainPose.getX() - translation.getX());
    double yDist = Math.abs(drivetrainPose.getY() - translation.getY());
    SmartDashboard.putNumber("drivetrain/auto-alignment-xdist", xDist);
    SmartDashboard.putNumber("drivetrain/auto-alignment-ydist", yDist);
    return ((xDist < VisionConstants.VISION_ALIGNMENT_DISCARD) && (yDist < VisionConstants.VISION_ALIGNMENT_DISCARD));
  }
}

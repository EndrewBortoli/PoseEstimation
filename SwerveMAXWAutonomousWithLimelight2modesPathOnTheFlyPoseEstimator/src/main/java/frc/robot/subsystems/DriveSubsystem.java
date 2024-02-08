// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.alignConstraints;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
// import frc.robot.Constants.limelightConstants;
// import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.subsystems.vision.LimelightObject;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private Field2d m_field = new Field2d();

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      true);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      false);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      true);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      false);

  // The gyro sensor
  private final static Pigeon2 m_gyro = new Pigeon2(8);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });


  //Variaveis criadas para serem usadas no método autoAlign();
  private double velCarangueijo = 0;
  private double velRotation = 0;
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

          //autobuilder needs to be configured last, add anything before this
          AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            //^These commands need to be created

            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(9.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(8.0, 0.0, 0.0), // Rotation PID constants
                    6, // Max module speed, in m/s
                    0.50, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
            
  );

  //45
  //50

  SmartDashboard.putData("Field", m_field);


  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
        m_field.setRobotPose(m_odometry.getPoseMeters());


        SmartDashboard.putNumber("RobotAngle", m_odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("velRotation", velRotation);
        SmartDashboard.putNumber("velCarangueijo", velCarangueijo);

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
   public Pose2d getPose() {
     return m_odometry.getPoseMeters();
   }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
         Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;


    System.out.println("Converted Velocities - X: " + xSpeedDelivered + ", Y: " + ySpeedDelivered + ", Rot: " + rotDelivered);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees((m_gyro.getRotation2d().getDegrees())))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  public void rotate(double rotation) {
    // Converta a velocidade de rotação desejada em radianos por segundo
    double rotationalSpeed = rotation * DriveConstants.kMaxAngularSpeed;
  
    // Configura o estado de módulo de cada roda para girar
    SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(rotationalSpeed));

    SwerveModuleState[] states = {desiredState, desiredState, desiredState, desiredState};
    setModuleStates(states);
  
    
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
   public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();  }
     /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */

   public ChassisSpeeds getChassisSpeeds() {
    final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(DriveConstants.globalxSpeed, DriveConstants.globalySpeed, DriveConstants.globalRot);
    return desiredSpeeds;
  }

  //see drive constants for details
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  
  public void goLeft() {
    double angle = Math.PI / 2.0; // 90 graus em radianos

    m_frontLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
    m_frontRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
    m_rearRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
}

  public void goRight() {
    double angle = Math.PI / -2.0; // 90 graus em radianos

    m_frontLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
    m_frontRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
    m_rearRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond, new Rotation2d(angle)));
}
  

public void autoAlign2() {
  LimelightObject m_LimelightObject = LimelightObject.getInstance();  // Use the existing instance

  PIDController rotationPID = null;  

  double rotationOffset = -72;

  final SlewRateLimiter xLimiter, rotLimiter;

  rotLimiter = m_rotLimiter;
  xLimiter = m_magLimiter;

  // Controles PID para se alinhar
  rotationPID = new PIDController(0.03, 0, 0);

  double velCarangueijo = 0;
  double velRotation = 0;

  if (m_LimelightObject.hasObject()) {
      velRotation = -rotationPID.calculate(m_LimelightObject.getYaw(), rotationOffset);
      System.out.println("Se alinhando");
  } else if(m_LimelightObject.hasObject() == false){
      velCarangueijo = 0.0;
      velRotation = 0.4;
      System.out.println("Fazendo nada");
  } else {
      velCarangueijo = 0.0;
      velRotation = 0.0;
      System.out.println("chegou no meio");
  }

  // 2. Apply deadband
  velRotation = Math.abs(velRotation) > OIConstants.kDriveDeadband ? velRotation : 0.0;

  // 3. Make the driving smoother
  velRotation = rotLimiter.calculate(velRotation) * 5;

  // Construct desired chassis speeds
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, velCarangueijo, velRotation);

  // Set chassis speeds using existing method
  setChassisSpeeds(chassisSpeeds);
}



public void autoAlign() {
  LimelightObject m_LimelightObject = LimelightObject.getInstance();  // Use the existing instance

  PIDController rotationPID = null;  
  PIDController carangueijoPID = null;

  double rotationOffset = -72;
  double carangueijoOffset = 1;

  final SlewRateLimiter xLimiter, rotLimiter;

  rotLimiter = m_rotLimiter;
  xLimiter = m_magLimiter;

  // Controles PID para se alinhar
  carangueijoPID = new PIDController(0.08, 0, 0);
  rotationPID = new PIDController(0.04, 0, 0);

  double velCarangueijo = 0;
  double velRotation = 0;

  if (m_LimelightObject.hasObject()) {
      velCarangueijo = carangueijoPID.calculate(m_LimelightObject.getXLimelight(), carangueijoOffset);
      velRotation = -rotationPID.calculate(m_LimelightObject.getYaw(), rotationOffset);
      System.out.println("Se alinhando");
  } else if(m_LimelightObject.hasObject() == false) {
      velCarangueijo = 0.0;
      velRotation = 0.4;
      System.out.println("Fazendo nada");
  } else {
      velCarangueijo = 0.0;
      velRotation = 0.0;
      System.out.println("Se alinhou");
  }

  // 2. Apply deadband
  velCarangueijo = Math.abs(velCarangueijo) > OIConstants.kDriveDeadband ? velCarangueijo : 0.0;
  velRotation = Math.abs(velRotation) > OIConstants.kDriveDeadband ? velRotation : 0.0;

  // 3. Make the driving smoother
  velCarangueijo = xLimiter.calculate(velCarangueijo) * 3;
  velRotation = rotLimiter.calculate(velRotation) * 5;

  // Construct desired chassis speeds
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, velCarangueijo, velRotation);

  // Set chassis speeds using existing method
  setChassisSpeeds(chassisSpeeds);
}


public void SpeakerLaunch(){
  AutoBuilder.pathfindToPose(
          new Pose2d(FieldConstants.BLUE_A1_1_FRONTSPEAKER_X, FieldConstants.BLUE_A1_1_FRONTSPEAKER_Y, Rotation2d.fromDegrees(FieldConstants.BLUE_A1_1_FRONTSPEAKER_DEGREES)), 
          new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          2.0
        );
}
public void Podium(){
  AutoBuilder.pathfindToPose(
          new Pose2d(FieldConstants.BLUE_PODIUM_X, FieldConstants.BLUE_PODIUM_Y, Rotation2d.fromDegrees(FieldConstants.BLUE_PODIUM_DEGREES)), 
          new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          2.0
        );
}
public void Amp(){
  AutoBuilder.pathfindToPose(
          new Pose2d(FieldConstants.BLUE_AMP_X, FieldConstants.BLUE_AMP_Y, Rotation2d.fromDegrees(FieldConstants.BLUE_AMP_DEGREES)), 
          new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          2.0
        );
}
public void Source(){
  AutoBuilder.pathfindToPose(
          new Pose2d(FieldConstants.BLUE_SOURCE_X, FieldConstants.BLUE_SOURCE_Y, Rotation2d.fromDegrees(FieldConstants.BLUE_SOURCE_DEGREES)), 
          new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          2.0
        );
}
public void Stage(){
  AutoBuilder.pathfindToPose(
          new Pose2d(FieldConstants.BLUE_B3_STAGE_X, FieldConstants.BLUE_B3_STAGE_Y, Rotation2d.fromDegrees(FieldConstants.BLUE_B3_STAGE_DEGREES)), 
          new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          2.0
        );
}


public void Test01(){
  AutoBuilder.pathfindToPose(
          new Pose2d(4.8, 2.4, Rotation2d.fromDegrees(0)), 
          new PathConstraints(
            1.0, 1.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          ), 
          0, 
          0.0
        );
}

}





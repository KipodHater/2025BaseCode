package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.io.PrintStream;
import java.util.function.*;


import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants;

import static frc.robot.Constants.ControllerConstants.STICK_DEADBAND;
import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystem extends SubsystemBase {
    // public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private PIDController aprilTagPIDController;

    private ChassisSpeeds currChassisSpeeds;
    private final PIDController xController = new PIDController(1, 10, 10);
    private final PIDController yController = new PIDController(1, 10, 10);
    private final PIDController headingController = new PIDController(1, 10, 10);

  Field2d field;
    public SwerveSubsystem() {
        gyro = new AHRS();
        field = new Field2d();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d(1,1,new Rotation2d(1,1)));
        currChassisSpeeds = new ChassisSpeeds();
        headingController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        currChassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getHeading())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(currChassisSpeeds);
        setModuleStates(swerveModuleStates);        
    }

    public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier angularSpeed,
            BooleanSupplier isFieldOriented) {
        return new RunCommand(() ->

        drive(
            new Translation2d(
                MathUtil.applyDeadband(xSpeed.getAsDouble(), STICK_DEADBAND),
                MathUtil.applyDeadband(ySpeed.getAsDouble(), STICK_DEADBAND)).times(MAX_SPEED),
            MathUtil.applyDeadband(angularSpeed.getAsDouble(), STICK_DEADBAND)  * MAX_ANGULAR_VELOCITY
            ,
            isFieldOriented.getAsBoolean(),
            true),

        this);
    }

    public Command driveConstantSpeed(double x, double y, double rotations, double time) {
        return new RunCommand(() -> drive(new Translation2d(x, y), rotations, true, true), this)
                .withTimeout(time)
                .andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, true)));
    }

    public void driveForAuto(ChassisSpeeds chassisSpeeds) {
        
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroYaw()));
        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
        // System.out.println("SET POSE");
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetRotation(heading);
        // System.out.println("set heading!");
    }


    public void zeroHeading() {
        poseEstimator.resetRotation(new Rotation2d());
        // System.out.println("Zeroed heading");
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-(double) gyro.getFusedHeading()); // changed from getyaw to fused heading
        //why is this (double)?
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }


   

    public ChassisSpeeds getCurrentSpeeds() {
        return currChassisSpeeds;
    }

    public double getRobotOrientationForSpeaker(){
        double robotsOrientation = Math.signum(
            MathUtil.applyDeadband(getPose().getRotation().getDegrees(), 25));
        return robotsOrientation;
    }
    
    private void rotateRobot(double angularSpeed){
        drive(new Translation2d(
                MathUtil.applyDeadband(0, STICK_DEADBAND),
                MathUtil.applyDeadband(0, STICK_DEADBAND)).times(MAX_SPEED),
                MathUtil.applyDeadband(angularSpeed, STICK_DEADBAND) * MAX_ANGULAR_VELOCITY
                ,
                true,
                //!isFieldOriented.getAsBoolean(),
                true);
    }

    private void setPIDRotation(double distanceFromAprilTagAngle){
        rotateRobot(aprilTagPIDController.calculate(distanceFromAprilTagAngle, 0));
    }

    public Command alignRobotToAprilTag(DoubleSupplier angleRelativeToAprilTag){
        return new RunCommand(() -> setPIDRotation(angleRelativeToAprilTag.getAsDouble()), this);
    }
    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        currChassisSpeeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + xController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        driveForAuto(currChassisSpeeds);
    }


    
      StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructPublisher<ChassisSpeeds> chpublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    StructPublisher<Pose2d> posepublisher = NetworkTableInstance.getDefault()
    .getStructTopic("pose", Pose2d.struct).publish();

    @Override
    public void periodic() {
     poseEstimator.update(getGyroYaw(), getModulePositions());
      for (SwerveModule mod : mSwerveMods) {
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      }
    //   System.out.println(gyro.getYaw());
        publisher.set(getModuleStates());
        chpublisher.set(getCurrentSpeeds());
        posepublisher.set(getPose());
        SmartDashboard.putNumber("XKP", SmartDashboard.getNumber("XKP", 1));
        SmartDashboard.putNumber("YKP", SmartDashboard.getNumber("YKP", 1));
        SmartDashboard.putNumber("RKP", SmartDashboard.getNumber("RKP", 1));

        xController.setP(SmartDashboard.getNumber("XKP", 0));
        yController.setP(SmartDashboard.getNumber("YKP", 0));
        headingController.setP(SmartDashboard.getNumber("RKP", 0));
        // SmartDashboard.putNumber("gyro angle", getGyroYaw().getDegrees());

      //System.out.println(getRobotOrientationForSpeaker());
      // System.out.println(mSwerveMods[4].getPosition());
      }    
    
}


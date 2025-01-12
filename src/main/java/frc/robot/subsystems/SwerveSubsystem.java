package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.*;


import com.kauailabs.navx.frc.AHRS;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.SwerveCalculations;
import frc.robot.Constants;

import static frc.robot.Constants.ControllerConstants.STICK_DEADBAND;
import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private PIDController aprilTagPIDController;

    private ChassisSpeeds currChassisSpeeds;
    private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(1.25, 0.0, 0.0);

    private ChassisSpeeds lastChassisSpeeds;
    private double MAX_ACCEL_FORWARD; // dont forget to remove!!!

  Field2d field;
    public SwerveSubsystem() {
        gyro = new AHRS();
        field = new Field2d();

        MAX_ACCEL_FORWARD = 1;

        lastChassisSpeeds = new ChassisSpeeds(0, 0,0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        
        currChassisSpeeds = new ChassisSpeeds();
        headingController.enableContinuousInput(-Math.PI, Math.PI); //radians
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
        
        //add accel limits here!
        currChassisSpeeds = limitAccel(lastChassisSpeeds, currChassisSpeeds);
        lastChassisSpeeds = currChassisSpeeds;

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(currChassisSpeeds);
        

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);
        

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        for (SwerveModuleState state : swerveModuleStates) {
            System.out.println(state.angle.getDegrees());
            // add logging here
        }
        
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
            isFieldOriented.getAsBoolean(), // removed the ! from this, check if works!!!
            true),

        this);
    }

    private ChassisSpeeds limitAccel(ChassisSpeeds iSpeeds, ChassisSpeeds fSpeeds){
        Translation2d forwardLimVector = 
        SwerveCalculations.calculateForwardLim(
            iSpeeds.vxMetersPerSecond, iSpeeds.vyMetersPerSecond,
            fSpeeds.vxMetersPerSecond, fSpeeds.vyMetersPerSecond, 
            MAX_ACCEL_FORWARD, MAX_SPEED);
        return new ChassisSpeeds(forwardLimVector.getX(), forwardLimVector.getY(), fSpeeds.omegaRadiansPerSecond); 
        // for now only works on forward lim!
    }

    public Command driveConstantSpeed(double x, double y, double rotations, double time) {
        return new RunCommand(() -> drive(new Translation2d(x, y), rotations, true, true), this)
                .withTimeout(time)
                .andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, true)));
    }

    public void driveForAuto(ChassisSpeeds chassisSpeeds) {
        
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(chassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroYaw()));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
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
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getGyroYaw();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }


    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-(double) gyro.getYaw()); // changed to fuzed heading
        //maybe minus uneeded?
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
        Pose2d pose = this.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + xController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        driveForAuto(speeds);
    }
//       StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
// .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
//     StructPublisher<ChassisSpeeds> chpublisher = NetworkTableInstance.getDefault()
//     .getStructTopic("Speeds", getCurrentSpeeds().struct).publish();

    @Override
    public void periodic() {
     swerveOdometry.update(getGyroYaw(), getModulePositions());
      for (SwerveModule mod : mSwerveMods) {
        
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);

          MAX_ACCEL_FORWARD = SmartDashboard.getNumber("Max Forward Accel m/s", 1);
          SmartDashboard.putNumber("Max Forward Accel m/s", MAX_ACCEL_FORWARD);

      }
    //   Logger.recordOutput("MySwerveModuleStates", getModuleStates());
    //   System.out.println(gyro.getYaw());
        // publisher.set(getModuleStates());
        // chpublisher.set(getCurrentSpeeds());

      //System.out.println(getRobotOrientationForSpeaker());
      // System.out.println(mSwerveMods[4].getPosition());
    }    
    
}


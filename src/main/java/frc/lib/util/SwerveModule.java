package frc.lib.util;

import static edu.wpi.first.units.Units.derive;

import java.lang.module.Configuration;
import java.security.Principal;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREConfigs;
import frc.robot.Constants;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward swerveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS,
    Constants.Swerve.DRIVE_KV,Constants.Swerve.DRIVE_KA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private final MotionMagicVoltage magicVoltage = new MotionMagicVoltage(0);

    private Rotation2d lastAngle;


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(CTREConfigs.swerveCANcoderConfig, 0.050);

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        angleMotor.getConfigurator().apply(CTREConfigs.swerveAngleFXConfig, 0.050);
        resetToAbsolute();
        System.out.println(CTREConfigs.swerveAngleFXConfig.Slot0.kP);

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotor.getConfigurator().apply(CTREConfigs.swerveDriveFXConfig, 0.050);
        driveMotor.getConfigurator().setPosition(0, 0.050);
    

        lastAngle = getState().angle;
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        System.out.println(angleOffset.getRotations() + " and " + getCANcoder().getRotations());
        System.out.println(absolutePosition);
        System.out.println(angleMotor.setPosition(absolutePosition));
        System.out.println(angleMotor.getPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);
        // anglePosition.Position = desiredState.angle.getRotations();
        // angleMotor.setControl(magicVoltage.withPosition(desiredState.angle.getRotations()));
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        anglePosition.Position = angle.getRotations();
        System.out.println(angle);
        // System.out.println(anglePosition.Position);
        angleMotor.setControl(anglePosition);

        lastAngle = angle;
    }

     private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.setControl(driveDutyCycle);

        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = swerveFeedforward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

      public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble())
        );
    }

}

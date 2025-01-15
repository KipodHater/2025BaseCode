package frc.robot;

import java.util.Optional;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Autonomous extends SubsystemBase {
    private final SwerveSubsystem drive;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public Autonomous(){
        drive = new SwerveSubsystem();
        autoChooser = new AutoChooser();
       

        autoFactory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            false,
            drive,
            new AutoBindings()
        );


        autoChooser.addRoutine("aaa", this::followPathAuto);
        autoChooser.select("aaa");
        SmartDashboard.putData("Routine" ,autoChooser);
    }
    
    
    

    

    public AutoRoutine followPathAuto(){
        AutoRoutine routine = autoFactory.newRoutine("testroutine");
        AutoTrajectory traj = routine.trajectory("New Path");

        
        System.out.println( traj.getInitialPose());
        System.out.println(drive.getPose());
        routine.active().onTrue(
            Commands.sequence(
                
                routine.resetOdometry(traj),
                traj.cmd()
                // Commands.print("Finished Auto")
            )
        );

        return routine;
            
        // );
        // return Commands.sequence(
        //     autoFactory.resetOdometry("New Path"),
        //     follow.
        // );
    }


    
    public Command getSelected(){
       return autoChooser.selectedCommandScheduler();
    }

    public void periodic() {

    }


}

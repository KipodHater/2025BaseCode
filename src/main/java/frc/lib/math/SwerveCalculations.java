package frc.lib.math;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveCalculations {
    
    public static Translation2d calculateForwardLim(double vxi, double vyi, double vxf, double vyf, double maxForwardAccel, double maxSpeed){
        double iAngleRad = Math.atan(vyi/vxi);
        double fAngleRad = Math.atan(vyf/vxf);

        double iMag = Math.sqrt(vxi*vxi + vyi*vyi);
        double fMag = Math.sqrt(vxf*vxf + vyf*vyf);

        double fMagProjection = Math.cos(fAngleRad - iAngleRad) * fMag;

        double currMaxAccel = maxForwardAccel * (1-iMag/maxSpeed);

        if(Math.abs(iMag - fMagProjection) < currMaxAccel * 0.02){
            return new Translation2d(vxf, vyf);
        }
        else{
            double targetMag = (iMag + (fMagProjection>iMag ? (-1):1) * currMaxAccel * 0.02)/Math.cos(fAngleRad - iAngleRad);
            return new Translation2d(targetMag, new Rotation2d(fAngleRad));
        }
    }
}

package frc.robot.controllers.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.interfaces.DriverInterface;

public class GenericController implements DriverInterface {
    private GenericHID controller;
    
    public GenericController(int id) {
        controller = new GenericHID(id);

    }

    @Override
    public Trigger isDriving() {
        if (getLeftX().getAsDouble() != 0 ||
        getLeftY().getAsDouble() != 0 ||   
        getRightX().getAsDouble() != 0){
            return new Trigger(() -> true); 
        }
        else {return new Trigger(() -> false);}
       
    }

    @Override
    public Trigger getA() {
        
        return new Trigger(() -> controller.getRawButton(1));
    }
    
    @Override
    public DoubleSupplier getLeftX() {
        
        return () -> -controller.getRawAxis(0);
    }

    @Override
    public DoubleSupplier getLeftY() {
        
        return () -> controller.getRawAxis(1);
    }

    @Override
    public DoubleSupplier getRightX() {
        
        return () -> -controller.getRawAxis(4);
    }
}

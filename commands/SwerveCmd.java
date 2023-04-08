package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCmd extends CommandBase{
    private final SwerveSubsystem swerve;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
    private final Supplier<Boolean> fieldOriented;
    private final SlewRateLimiter xLim, yLim, thetaLim;

    public SwerveCmd(SwerveSubsystem swerve, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction, Supplier<Boolean> fieldOriented) {
        this.swerve = swerve;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turnSpdFunction = turnSpdFunction;
        this.fieldOriented = fieldOriented;
        this.xLim = new SlewRateLimiter(DriverConstants.maxAccel);
        this.yLim = new SlewRateLimiter(DriverConstants.maxAccel);
        this.thetaLim = new SlewRateLimiter(DriverConstants.maxTurnAccel);
        addRequirements(swerve);
    }
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpd = xSpdFunction.get();                                               //acquire joystick inputs
        double ySpd = ySpdFunction.get();
        double thetaSpd = turnSpdFunction.get();

        xSpd = Math.abs(xSpd) > DriverConstants.deadband ? xSpd : 0.0;                  //deadband application
        ySpd = Math.abs(ySpd) > DriverConstants.deadband ? ySpd : 0.0;
        thetaSpd = Math.abs(thetaSpd) > DriverConstants.deadband ? thetaSpd : 0.0;

        xSpd = xLim.calculate(xSpd) * ModuleConstants.maxSpeed;                                                    //Slew Limiter application
        ySpd = yLim.calculate(ySpd) * ModuleConstants.maxSpeed;
        thetaSpd = thetaLim.calculate(thetaSpd) * ModuleConstants.maxTurnRate;

        ChassisSpeeds inputStates;                                                                              //convert joystick inputs into chassis velocities
        if(fieldOriented.get()) {
            inputStates = ChassisSpeeds.fromFieldRelativeSpeeds(xSpd, ySpd, thetaSpd, swerve.getRotation2d());
        } else {
            inputStates = new ChassisSpeeds(xSpd, ySpd, thetaSpd);
        }

        SwerveModuleState[] moduleStates = ChassisConstants.kKinematics.toSwerveModuleStates(inputStates);      //convert chassis velocities into states for each module

        swerve.setModuleStates(moduleStates);


    }

   
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

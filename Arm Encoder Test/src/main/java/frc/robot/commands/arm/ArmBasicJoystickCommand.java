package frc.robot.commands.arm;

import java.util.function.Supplier;

import javax.swing.text.Utilities;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class ArmBasicJoystickCommand extends CommandBase {

    Arm m_arm;
    Supplier<CommandXboxController> joystick;


    public ArmBasicJoystickCommand(Arm arm, Supplier<CommandXboxController> joystick) {
        m_arm = arm;
        this.joystick = joystick;
        addRequirements(m_arm);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void execute() {
        m_arm.setShoulderSpeed(deadband(-joystick.get().getLeftY(), 0.2));
        m_arm.setElbowSpeed(deadband(-joystick.get().getRightY(), 0.2));
        
    }

    public double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance) {
            return 0.0;
        }
        else {
            return value;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}

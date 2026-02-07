package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase
{

    public Command exampleMethodCommand()
    {
        return new Command()
        {

        };
    }

    // public BooleanSupplier exampleCondition(){
    // return new BooleanSupplier() {
    // @Override
    // public boolean getAsBoolean(){
    // return false;
    // }
    // };
    // }

    // public BooleanSupplier exampleCondition = new BooleanSupplier() {
    // @Override
    // public boolean getAsBoolean() {
    // return false;
    // };
    // };

    public boolean exampleCondition()
    {
        return false;
    }
}

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareConstants;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.Animation;


public class LEDs extends SubsystemBase {

    CANdle _candle;
    static int _numLEDs = 5;
    boolean _alreadyRunning = true;
    LEDAnimation _currentAnimation = LEDAnimation.None;


    public enum LEDAnimation {
        None(null, null, 0),

        // RobotIdle(new LEDColor(0, 0, 255), null, 0),

        BlinkGreen(new LEDColor(0, 255, 0), null, 3),
        
        // SolidGreen(new LEDColor(0, 255, 0), null, 0),

        BlinkDarkBlue(null, new StrobeAnimation(0, 0, 150, 1, 0.5, _numLEDs), 3),

        SolidDarkBlue(new LEDColor(0, 0, 150), null, 0);

        // PartyMode(null, new RainbowAnimation(100, 0.5, _numLEDs), 0),
   
        // SolidTeal(new LEDColor(0,225,174), null, 0),

        // SolidCoral(new LEDColor(250, 130, 120), null, 0);

        
        LEDColor _color;
        Animation _animation;
        int _secondsToRun;

        LEDAnimation(LEDColor color, Animation animation, int secondsToRun) {
            _color = color;
            _animation = animation;
            _secondsToRun = secondsToRun;
        }

        public LEDColor getColor() {
            return _color;
        }

        public Animation getAnimation() {
            return _animation;
        }

        public int getSecondsToRun() {
            return _secondsToRun;
        }
    }

    public void runAnimation(LEDAnimation animation) {
        if (animation != _currentAnimation) {
            _currentAnimation = animation;

            if (animation.getColor() == null) {
                _candle.clearAnimation(0);
                _candle.setLEDs(0,0,0);
                _candle.animate(animation.getAnimation());
            } else if (animation.getAnimation() == null){
                LEDColor color = animation.getColor();
                _candle.clearAnimation(0);
                _candle.setLEDs(0,0,0);     
                _candle.setLEDs(color.getR(), color.getG(), color.getB());
            } else {
                _candle.clearAnimation(0);
                _candle.animate(animation.getAnimation());
                LEDColor color = animation.getColor();
                _candle.setLEDs(color.getR(), color.getG(), color.getB());
            }
            
        }
    }

    public void intaking() {
        if(!_alreadyRunning) {
            runAnimation(LEDAnimation.BlinkGreen);
            _alreadyRunning = false;
        }
    }

    public void elevatorOrArmIsMoving() {
        if(!_alreadyRunning) {
            runAnimation(LEDAnimation.BlinkDarkBlue);
            _alreadyRunning = false;
        }
    }

    public void elevatorAndArmAtSetpoints() {
        if(!_alreadyRunning) {
            runAnimation(LEDAnimation.SolidDarkBlue);
            _alreadyRunning = false;
        }
    }

    public void reset() {
        _alreadyRunning = false;
    }
}

#include "mbed.h"
#include "LCDi2c.h"

// =============================================================================
// OVERVIEW & ARCHITECTURE
// =============================================================================
//
// This program is designed to act as a turret controller using the K66F MCU.
// It has several main functions:
//
// 1. **Serial Communication:**  
//    The MCU reads commands from a USB serial port. Commands (characters '0' to '5')
//    instruct the turret to move left/right (X-axis) or up/down (Y-axis).
//
// 2. **Servo Control:**  
//    Two PWM outputs drive servo motors for the X-axis and Y-axis.
//      - servo_x (on pin PTC8) controls the horizontal (left/right) motion.
//      - servo_y (on pin PTC5) is reserved for the vertical motion, although in this 
//        code it is not updated via modulation because we are dedicating another PWM 
//        (modulation_pwm) to the ultrasonic system.
//    The servo angles are converted to PWM pulse widths between MIN_PULSE and MAX_PULSE.
//
// 3. **Ultrasonic System:**  
//    Two PWM outputs are dedicated to the ultrasonic system.
//      - ultrasonic_pwm (on pin PTB18) generates the ultrasonic carrier signal at 
//        40kHz. A 50% duty cycle produces a square wave ideal for driving ultrasonic
//        transducers.
//      - modulation_pwm (on pin PTC2) produces a lower frequency (nominally 1kHz)
//        modulation signal. This signal can be used to amplitude modulate the carrier,
//        effectively turning the output on and off at the modulation rate.
//        *Note: The period is set to 3ms (3.0f / 1000.0f), which corresponds to ~333Hz.
//         To get 1kHz, the period should be 1.0f/1000.0f. Adjust as needed.*
//
// 4. **LCD Output:**  
//    An I2C-connected LCD (16x2) is used to display the current command/direction.
//    It gives visual feedback (for example, "Moving Left", "Moving Up", etc.)
//
// 5. **Main Loop Operation:**  
//    The main loop continuously checks for serial input commands and then updates:
//      - The servo positions if a move command is active.
//      - The LCD status display.
//    Meanwhile, the ultrasonic PWM outputs run in hardware independently.
// -----------------------------------------------------------------------------

// =============================================================================
// SETUP OF PERIPHERALS
// =============================================================================

// Serial communication over USB (appears as /dev/ttyACM0 on your system)
BufferedSerial serial_port(USBTX, USBRX, 9600);

// PWM outputs for servo control:
// servo_x drives the horizontal (X-axis) servo motor on pin PTC8.
PwmOut servo_x(PTC8);
// servo_y drives the vertical (Y-axis) servo motor on pin PTC5.
// *Note: A comment suggests that the modulation output might replace a servo on PTC2,
// but here servo_y remains on PTC5 so that the ultrasonic modulation uses a separate pin.*
PwmOut servo_y(PTC5);

// PWM outputs for the ultrasonic system:
// ultrasonic_pwm generates the 40kHz carrier wave on pin PTB18.
PwmOut ultrasonic_pwm(PTB18);
// modulation_pwm generates the lower frequency modulation signal (nominally 1kHz)
// on pin PTC2. This modulates the amplitude of the ultrasonic signal.
PwmOut modulation_pwm(PTC2);

// LCD setup via I2C for visual feedback (16x2 LCD)
LCDi2c lcd(I2C_SDA, I2C_SCL, LCD16x2); 

// =============================================================================
// SERVO CONFIGURATION PARAMETERS
// =============================================================================
const float PWM_PERIOD = 0.020f;   // 20ms period for servo control (50 Hz)
const float MIN_PULSE  = 0.001f;     // Minimum pulse width: 1ms (represents 0°)
const float MAX_PULSE  = 0.002f;     // Maximum pulse width: 2ms (represents 180°)

// =============================================================================
// GLOBAL VARIABLES: SERVO STATE
// =============================================================================
// Current angle and movement direction for the X-axis servo (horizontal)
float current_angle_x = 90.0f; // Starting at neutral position (90°)
int direction_x = 0;          // Direction: 0 = stop, -1 = left, +1 = right

// Current angle and movement direction for the Y-axis servo (vertical)
float current_angle_y = 90.0f; // Starting at neutral position (90°)
int direction_y = 0;          // Direction: 0 = stop, -1 = down, +1 = up

// The incremental step (in degrees) for each update command.
const float STEP = 10.0f;

// =============================================================================
// FUNCTION: set_servo_angle
// ----------------------------------------------------------------------------
// Converts a given angle (0° to 180°) into a pulse width and applies it to a
// given PWM output. This function ensures that the servo moves to the corresponding
// angle using the MIN_PULSE and MAX_PULSE settings.
//
// Parameters:
//   - servo: The PWM output controlling the servo motor.
//   - angle: The desired angle (in degrees).
// =============================================================================
void set_servo_angle(PwmOut &servo, float angle) {
    if (angle < 0.0f) angle = 0.0f;
    else if (angle > 180.0f) angle = 180.0f;
    
    // Map the angle to the appropriate pulse width between MIN_PULSE and MAX_PULSE.
    float pulse_width = MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE);
    servo.pulsewidth(pulse_width);
}

// =============================================================================
// MAIN FUNCTION
// =============================================================================
int main() {
    // -------------------------------------------------------------------------
    // Servo Initialization
    // -------------------------------------------------------------------------
    // Set the PWM period for the servo on the X-axis.
    servo_x.period(PWM_PERIOD);
    // Immediately set servo_x to 0° (or a starting calibration position).
    set_servo_angle(servo_x, 0);
    // Wait for 200 milliseconds to allow servo initialization.
    ThisThread::sleep_for(200ms);
    // Move servo_x to the neutral starting position.
    set_servo_angle(servo_x, current_angle_x);

    // -------------------------------------------------------------------------
    // Ultrasonic System Initialization
    // -------------------------------------------------------------------------
    // Configure the ultrasonic PWM for a 40kHz carrier signal.
    // Calculation: period = 1 / frequency = 1 / 40000 = 25 microseconds.
    ultrasonic_pwm.period(1.0f / 40000.0f); // 40kHz
    // Set a 50% duty cycle for a symmetrical square wave (ideal for most transducers).
    ultrasonic_pwm.write(0.5f);

    // Initialize the modulation PWM.
    // *Note: Current period is set to 3.0f/1000.0f, which equals 3ms period (~333Hz).
    // For a true 1kHz modulation, use period = 1.0f / 1000.0f (1ms).
    modulation_pwm.period(3.0f / 1000.0f); // Set period; adjust to 1.0/1000 for 1kHz if desired.
    // Set the modulation PWM duty cycle. A value of 0.7f indicates that the output
    // is "on" 70% of the time; you can adjust this to control the modulation depth.
    modulation_pwm.write(0.7f);

    // -------------------------------------------------------------------------
    // LCD and Serial Initialization
    // -------------------------------------------------------------------------
    // Print an initial message over serial.
    printf("Turret controller started.\nX-axis angle: %.1f°\nY-axis angle: %.1f°\n", 
           current_angle_x, current_angle_y);
    
    // -------------------------------------------------------------------------
    // MAIN LOOP: PROCESS SERIAL COMMANDS AND UPDATE SERVO POSITIONS
    // -------------------------------------------------------------------------
    char buf[1];  // Buffer for receiving one character at a time over serial.
    while (true) {
        // Check if there is any data available on the serial port.
        if (serial_port.readable()) {
            serial_port.read(buf, 1);   // Read one character command.
            printf("Received command: %c\n", buf[0]);

            // Process received commands to adjust the turret direction:
            if (buf[0] == '0') {
                direction_x = -1; // Move left on X-axis.
                printf("X-axis: Moving left\n");
                lcd.cls();
                lcd.locate(0, 0);
                lcd.printf("Moving Left");
            } else if (buf[0] == '1') {
                direction_x = +1; // Move right on X-axis.
                printf("X-axis: Moving right\n");
                lcd.cls();
                lcd.locate(0, 0);
                lcd.printf("Moving Right");
            } else if (buf[0] == '2') {
                direction_x = 0; // Stop X-axis movement.
                printf("X-axis: Stopped\n");
            } else if (buf[0] == '3') {
                direction_y = +1; // Move up on Y-axis.
                printf("Y-axis: Moving up\n");
                lcd.cls();
                lcd.locate(0, 0);
                lcd.printf("Moving Up");
            } else if (buf[0] == '4') {
                direction_y = -1; // Move down on Y-axis.
                printf("Y-axis: Moving down\n");
                lcd.cls();
                lcd.locate(0, 0);
                lcd.printf("Moving Down");
            } else if (buf[0] == '5') {
                direction_y = 0; // Stop Y-axis movement.
                printf("Y-axis: Stopped\n");
            }
        }

        // ---------------------------------------------------------------------
        // Update Servo Positions Based on Direction Commands
        // ---------------------------------------------------------------------
        // For the X-axis: update current_angle_x based on the direction (if moving).
        if (direction_x != 0) {
            current_angle_x += direction_x * STEP;
            // Clamp the angle to the valid range [0, 180]:
            if (current_angle_x <= 0.0f) {
                current_angle_x = 0.0f;
                direction_x = 0;
                printf("X-axis: Reached 0° limit.\n");
            } else if (current_angle_x >= 180.0f) {
                current_angle_x = 180.0f;
                direction_x = 0;
                printf("X-axis: Reached 180° limit.\n");
            }
            // Update the servo position.
            set_servo_angle(servo_x, current_angle_x);
            printf("X-axis current angle: %.1f°\n", current_angle_x);
        }

        // For the Y-axis: update current_angle_y based on the direction (if moving).
        if (direction_y != 0) {
            current_angle_y += direction_y * STEP;
            // Clamp the angle to the valid range [0, 180]:
            if (current_angle_y <= 0.0f) {
                current_angle_y = 0.0f;
                direction_y = 0;
                printf("Y-axis: Reached 0° limit.\n");
            } else if (current_angle_y >= 180.0f) {
                current_angle_y = 180.0f;
                direction_y = 0;
                printf("Y-axis: Reached 180° limit.\n");
            }
            // In this version, the Y-axis servo output (servo_y) is not updated by set_servo_angle,
            // as that function call is commented out. You could enable it if you're using a servo on that channel.
            // set_servo_angle(servo_y, current_angle_y);
            printf("Y-axis current angle: %.1f°\n", current_angle_y);
        }

        // ---------------------------------------------------------------------
        // Wait a short period before the next loop iteration
        // This helps pace the servo updates and reduces CPU usage.
        ThisThread::sleep_for(20ms);
    }
}

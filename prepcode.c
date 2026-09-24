/*
 * Main.c
 *
 * Main file containing the main state machine.
 */
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "Library/Reflectance.h"
#include "Library/Clock.h"
#include "Library/Bump.h"
#include "Library/Motor.h"
#include "Library/Encoder.h"
#include "Library/Button.h"

void Initialize_System();

uint8_t light_data; // QTR-8RC
uint8_t light_data0;
uint8_t light_data1;
uint8_t light_data2;
uint8_t light_data3;
uint8_t light_data4;
uint8_t light_data5;
uint8_t light_data6;
uint8_t light_data7;

int32_t Position; // 332 is right, and -332 is left of center
uint8_t bump_data;
uint8_t bump_data0;
uint8_t bump_data2;
uint8_t bump_data3;
uint8_t bump_data5;

int tick=0;

int mytime[20];
int i=0;

typedef enum
{
    START = 0,
    WAIT,
    Left90,
    checkLeft90,
    Right90,
    checkRight90,
    BUMMPED1a,
    BUMMPED1b,
    BUMMPED2a,
    BUMMPED2b,
    DRIVE,
    BACKUP,
    LIGHTCHECK,
    STOP

} my_state_t;

my_state_t state = START;

int main(void)
{
    bool done;

    Initialize_System();

    Reflectance_Init();

    Bump_Init();

    motor_init();

    encoder_init();

    button_init();

    set_left_motor_pwm(0);
    set_right_motor_pwm(0);

    while (1)
    {
        // Read Reflectance data into a byte.
        // Each bit corresponds to a sensor on the light bar
        light_data = Reflectance_Read(1000);
        light_data0 = LIGHT_BAR(light_data,0);
        light_data1 = LIGHT_BAR(light_data,1);
        light_data2 = LIGHT_BAR(light_data,2);
        light_data3 = LIGHT_BAR(light_data,3);
        light_data4 = LIGHT_BAR(light_data,4);
        light_data3 = LIGHT_BAR(light_data,3);
        light_data4 = LIGHT_BAR(light_data,4);
        light_data5 = LIGHT_BAR(light_data,5);
        light_data6 = LIGHT_BAR(light_data,6);
        light_data7 = LIGHT_BAR(light_data,7);
        // Convert light_data into a Position using a weighted sum
        Position = Reflectance_Position(light_data);

        // Read Bump data into a byte
        // Lower six bits correspond to the six bump sensors
        // put into individual variables so we can view it in GC
        bump_data = Bump_Read();
        bump_data0 = BUMP_SWITCH(bump_data,0);
        bump_data2 = BUMP_SWITCH(bump_data,2);
        bump_data3 = BUMP_SWITCH(bump_data,3);
        bump_data5 = BUMP_SWITCH(bump_data,5);

        // Emergency stop switch S2
        // Switch to state "STOP" if pressed
        if (button_S2_pressed()) state = STOP;

        //-----------------------------------
        //        Main State Machine
        //-----------------------------------
        switch (state) {

        case START:
                state = WAIT;
        break;

        case WAIT:
            if (button_S1_pressed()) {
                state = DRIVE;
            }
        break;

        case DRIVE:

           set_left_motor_direction(true);
           set_right_motor_direction(true);
           set_left_motor_pwm(.25);
           set_right_motor_pwm(.25);

           if(bump_data2 && bump_data3) state = BACKUP;
           if(bump_data0) state = BUMMPED1a;
           if(bump_data5) state = BUMMPED2a;

        break;


        case Left90:
            rotate_motors_by_counts(INITIAL, .25, 540, 0);  // Turns Robot 90 degrees to the Left
            state = checkLeft90;
        break;

        case checkLeft90:
            if (bump_data0) state = BUMMPED1a;

            done = rotate_motors_by_counts(CONTINUOUS, .25, 540, 0);  // Continue to rotate until done

            if (done) state = DRIVE;
        break;

        case Right90:
             rotate_motors_by_counts(INITIAL, .25, 0, 540);  // Turns Robot 90 degrees to the Right
             state = checkLeft90;
        break;

        case checkRight90:
            if (bump_data5) state = BUMMPED2a;

            done = rotate_motors_by_counts(CONTINUOUS, .25, 0, 540);  // Continue to rotate until done

            if (done) state = DRIVE;
        break;

        case BUMMPED1a:
            set_left_motor_pwm(0);          // Stop all motors
            set_right_motor_pwm(0);

            rotate_motors_by_counts(INITIAL, .25, -200, 0);
            state = BUMMPED1b;
        break;

        case BUMMPED1b:
           done = rotate_motors_by_counts(CONTINUOUS, .25, -200, 0);

           if (done) state = DRIVE;

        break;

        case BUMMPED2a:
             set_left_motor_pwm(0);          // Stop all motors
             set_right_motor_pwm(0);

             rotate_motors_by_counts(INITIAL, .25, 0, -200);
             state = BUMMPED2b;

        break;

        case BUMMPED2b:
             done = rotate_motors_by_counts(CONTINUOUS, .25, 0, -200);

             if (done) state = DRIVE;
        break;


        case BACKUP:

            set_left_motor_direction(-360);
            set_right_motor_direction(-360);
            set_left_motor_pwm(.25);
            set_right_motor_pwm(.25);

            state = Left90;

        break;

        case LIGHTCHECK:


        Clock_Delay1ms(1000);
        if (light_data0 && light_data7 == 1)
        set_left_motor_pwm(0);          // Stop all motors
        set_right_motor_pwm(0);
        if (light_data0 && light_data7 == 0) state = DRIVE;


        break;



        case STOP:

            set_left_motor_pwm(0);          // Stop all motors
            set_right_motor_pwm(0);
        break;
        }

        Clock_Delay1ms(10);
    }
}

void Initialize_System()
{
    /*
     * Initialize main clock
     *
     * SMCLK = 12Mhz
     */
    Clock_Init48MHz();

    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    /* Configuring GPIO LED1 as an output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Configure GPIO LED Red, LED Green, LED Blue */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    /*
     * Configuring SysTick to trigger at .001 sec (MCLK is 48Mhz)
     */
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(48000);
    MAP_SysTick_enableInterrupt();

    MAP_Interrupt_enableMaster();
}


/*
 * Handle the SysTick Interrupt.  Currently interrupting at 1/10 second.
 *
 * Increment the tick counter "tick"
 * Blink the red led
 */
void SysTick_Handler(void)
{
    tick++;
    if ((tick%1000)==0) MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);        // Toggle RED LED each time through loop
}





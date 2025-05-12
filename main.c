#include <math.h>
int altitude;
int altitude_min;
int altitude_max;

int altitude_potentiometer;
int roll_potentiometer;
float motor_speed, left_motor_speed, right_motor_speed;

/**
Read the value from a potentiometer connected to the specified channel
*/
float read_potentiometer(const int channel) {
    ADMUX = (1 << REFS0) | (channel & 0x0F); /* AVCC as reference, select ADC channel */
    ADCSRA |= (1 << ADSC); /* Start conversion */
    while (ADCSRA & (1 << ADSC)); /* Wait for conversion to finish */

    int value = ADCL + (ADCH << 8); /* Combine 10-bit result */
    return value;
}

/**
Scale an ADC value to a percentage
*/
float scale_adc_to_percent(const float value) {
    return ((float) value / 1023.0f) * 100.0f;
}

void control_altitude(const int altitude_potentiometer) {
    /* Modify altitude */
    // Control altitude with potentiometer
    if (altitude_potentiometer < 50) {
        altitude--;
    } else if (altitude_potentiometer > 50) {
        altitude++;
    }

    if (altitude < 0) {
        /* Quadcopter reached ground level */
        altitude = 0;
    } else if (altitude > altitude_max) {
        /* Quadcopter reached ceiling altitde */
        altitude = altitude_max;
    }
}

void display_altitude() {
    /* Get the digits of the altitude value */
    unsigned int n_altitude_digits = 3;
    int *altitude_digits = digits(altitude, n_altitude_digits);
    int units = altitude_digits[0];
    int tens = altitude_digits[1];
    int hundreds = altitude_digits[2];

    /* Get the bits of each digit (4 bits per digit) */
    unsigned int bits_display = 4;
    int *bits_units = decToBin(units, bits_display);
    int *bits_tens = decToBin(tens, bits_display);
    int *bits_hundreds = decToBin(hundreds, bits_display);

    /* Send bits in order: units, tens, hundreds */
    sendBits(bits_units, 4, 0, 1);
    sendBits(bits_tens, 4, 0, 1);
    sendBits(bits_hundreds, 4, 0, 1);

    /* Pulse STB to send data to display */
    PORTD |= (1 << 2);
    PORTD &= ~(1 << 2);

    /* Delay */
    // Control delay depending on motor speed (potentiometer value)
    for (volatile long delay = 0; delay < 10000; delay++) {}

    /* Free allocated memory */
    free(altitude_digits);
    free(bits_units);
    free(bits_tens);
    free(bits_hundreds);
}

/**
Split a number in digits and return as an array
*/
int *digits(int number, const int n_digits) {
    int *digits = (int *) malloc(n_digits * sizeof(int));
    /* Safety check */
    if (digits == NULL) return NULL;

    for (int i = 0; i < n_digits; i++) {
        digits[i] = number % 10;
        number /= 10;
    }
    return digits;
}


/**
Send an array of bits and clock pulse over PIND
*/
void sendBits(const int *bits, const int n_bits, const unsigned int channel_d, const unsigned int channel_clk) {
    /* Clear ports before setting new bits */
    PORTD &= ~((1 << channel_d) | (1 << channel_clk));
    /* Send bits to port */
    for (int i = n_bits - 1; i >= 0; i--) {
        if (bits[i]) {
            PORTD |= (1 << channel_d);
        } else {
            PORTD &= ~(1 << channel_d);
        }

        /* Clock pulse */
        PORTD |= (1 << channel_clk);
        PORTD &= ~(1 << channel_clk);
    }
}

/**
Extract the bits of a decimal number to an array
*/
int *decToBin(const int decimal, const int n_bits) {
    int *bits = (int *) malloc(n_bits * sizeof(int));
    for (int i = n_bits - 1; i >= 0; i--) {
        bits[n_bits - 1 - i] = (decimal >> i) & 1;
    }
    return bits;
}

void setup() {
    /* Configure I/O */
    DDRB = 0b00000000; /* Set [PB0-PB7] as input */
    DDRC = 0b00000000; /* Set [PC0-PD3] as output */
    DDRD = 0b00000111; /* Set [PD0-PD2] as output */
    /* Configure pull-up resistor */
    PORTC &= ~(1 << 0); /* Disable pull-up resistor on PC0 */
    PORTC &= ~(1 << 1); /* Disable pull-up resistor on PC1 */
    /* Set initial values */
    altitude_min = 0; /* Ground altitude of the quadcopter */
    altitude_max = 999; /* Ceiling altitude of the quadcopter */
    altitude = altitude_min; /* Altitude starts at ground level */
}

void loop() {
    /* If the controller is ON (PB0 high level) */
    if ((PINB & (1 << 0))) {
        altitude_potentiometer = scale_adc_to_percent(read_potentiometer(0));
        roll_potentiometer = scale_adc_to_percent(read_potentiometer(1));
        /* Total motor speed: Ms = 0.9Ap + 0.1Rp */
        float altitude_speed = 0.9f * (altitude_potentiometer - 50) * 2;
        float roll_speed = 0.1f * (roll_potentiometer - 50) * 2;

        /* Generate torque with the speed difference */
        if (roll_speed > 0) {
            left_motor_speed = altitude_speed + roll_speed;
            right_motor_speed = altitude_speed - roll_speed;
        } else if (roll_speed < 0) {
            left_motor_speed = altitude_speed - roll_speed;
            right_motor_speed = altitude_speed + roll_speed;
        } else {
            left_motor_speed = right_motor_speed = altitude_speed;
        }
        /* Motor speed in absolute value */
        motor_speed = altitude_speed + abs(roll_speed);

        control_altitude(motor_speed);
        display_altitude();
    }
}
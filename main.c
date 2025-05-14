#include <stdlib.h>
int current_altitude;
int altitude_lower_limit;
int altitude_upper_limit;

int current_roll_angle;
int roll_angle_min;
int roll_angle_max;

/* **************************************************** */
/* ******************* ENTRY POINT ******************** */
/* **************************************************** */

/**
 * Initializes I/O pins, pull-up resistors, and global state variables.
 *
 * Called once at system startup.
 */
void setup() {
	/* Configure I/O */
	DDRB = 0b00000000; /* Set [PB0-PB7] as input */
	DDRC = 0b00000000; /* Set [PC0-PD3] as output */
	DDRD = 0b00111111; /* Set [PD0-PD5] as output */
	/* Configure pull-up resistor */
	PORTC &= ~(1 << 0); /* Disable pull-up resistor on PC0 */
	PORTC &= ~(1 << 1); /* Disable pull-up resistor on PC1 */
	/* Set initial values */
	altitude_lower_limit = 0; /* Ground altitude of the quadcopter */
	altitude_upper_limit = 999; /* Ceiling altitude of the quadcopter */
	current_altitude = altitude_lower_limit; /* Altitude starts at ground level */

	roll_angle_min = -45;
	roll_angle_max = 45;
	current_roll_angle = roll_angle_min;
}

/**
 * Main loop function that:
 * - Reads potentiometer inputs
 * - Updates altitude and roll based on scaled inputs
 * - Updates the speed of the motors based on scaled inputs
 * - Displays current altitude and roll
 *
 * Executes only when controller switch (PB0) is ON.
 */
void loop() {
	/* If the controller is ON (PB0 high level) */
	if ((PINB & (1 << 0))) {
		const float pot_altitude = scale_adc_to_percent(read_potentiometer(0));
		const float pot_roll = scale_adc_to_percent(read_potentiometer(1));

		control_altitude((int) pot_altitude);
		control_roll((int) pot_roll);
		control_motors(pot_altitude, pot_roll);
		display_altitude();
		display_roll();
	}
}

/* ***************************************************** */
/* ******************* CONTROL LOGIC ******************* */
/* ***************************************************** */

static int altitude_accumulator = 0;
const int SCALE = 100;

/**
 * Adjusts the altitude based on the potentiometer input.
 * Simulates a climb rate by accumulating deviation from 50% input.
 *
 * @param pot_altitude_percentage Value from altitude potentiometer (0–100%).
 */
void control_altitude(const int pot_altitude_percentage) {
	/* Clamp input to [0, 100] */
	const int percent = (int) clamp((float) pot_altitude_percentage, 0, 100);
	/* Calculate climb with scaling */
	altitude_accumulator += percent - 50; // Range -50 to +50

	/* When accumulated enough for at least 1 unit */
	const int climb = altitude_accumulator / SCALE;
	altitude_accumulator %= SCALE;

	current_altitude += climb;
	/* Clamp altitude */
	current_altitude = (int) clamp((float) current_altitude, (float) altitude_lower_limit, (float) altitude_upper_limit);
}

/**
 * Updates the roll angle based on the roll potentiometer input.
 * Maps the percentage input linearly to the roll range [roll_min, roll_max].
 *
 * @param pot_roll_percentage Value from roll potentiometer (0–100%).
 */
void control_roll(const int pot_roll_percentage) {
	/* Clamp input to [0, 100] */
	const float percent = clamp((float) pot_roll_percentage, 0, 100);

	/* Linearly map percentage to roll */
	current_roll_angle = (int) ((float) roll_angle_min + ((float) (roll_angle_max - roll_angle_min) * percent) / 100.0f);
}

void control_motors(float speed_altitude, float speed_roll) {
	/* Security clamp */
	speed_altitude = clamp(speed_altitude, 0.0f, 100.0f);
	speed_roll = clamp(speed_roll, -45.0f, 45.0f);

	/* Left motors slow down and right motors speed up on positive roll values */
	const float speed_left = speed_altitude - speed_roll;
	const float speed_right = speed_altitude + speed_roll;
}

/* ******************************************************** */
/* ******************** INPUT HANDLING ******************** */
/* ******************************************************** */

/**
 * Reads the analog value from a potentiometer connected to the specified ADC channel.
 *
 * @param channel ADC channel number to read from.
 * @return The raw 10-bit analog value.
 */
float read_potentiometer(const int channel) {
	ADMUX = (1 << REFS0) | (channel & 0x0F); /* AVCC as reference, select ADC channel */
	ADCSRA |= (1 << ADSC); /* Start conversion */
	while (ADCSRA & (1 << ADSC)) /* Wait for conversion to finish */
		;

	const float value = ADCL + (ADCH << 8); /* Combine 10-bit result */
	return value;
}

/**
 * Converts a raw ADC value into a percentage.
 *
 * @param value Raw ADC value.
 * @return Percentage value scaled between 0.0 and 100.0.
 */
float scale_adc_to_percent(const float value) { return ((float) value / 1023.0f) * 100.0f; }

/* ********************************************************* */
/* ******************** OUTPUT HANDLING ******************** */
/* ********************************************************* */

/**
 * Displays the current altitude on a 3-digit digital display.
 * Converts the altitude into digits, then into bits, and sends them to the display.
 * Assumes external memory management and display hardware setup.
 *
 * Memory: Allocates and frees temporary arrays for digit/bit storage.
 */
void display_altitude() {
	/* Get the digits of the altitude value */
	unsigned int n_altitude_digits = 3;
	int *altitude_digits = extract_digits(current_altitude, n_altitude_digits);
	int units = altitude_digits[0];
	int tens = altitude_digits[1];
	int hundreds = altitude_digits[2];

	/* Get the bits of each digit (4 bits per digit) */
	unsigned int bits_display = 4;
	int *bits_units = decimal_to_binary(units, bits_display);
	int *bits_tens = decimal_to_binary(tens, bits_display);
	int *bits_hundreds = decimal_to_binary(hundreds, bits_display);

	/* Send bits in order: units, tens, hundreds */
	send_bits_to_display(bits_units, 4, 0, 1);
	send_bits_to_display(bits_tens, 4, 0, 1);
	send_bits_to_display(bits_hundreds, 4, 0, 1);

	/* Pulse STB to send data to display */
	PORTD |= (1 << 2);
	PORTD &= ~(1 << 2);

	/* Delay */
	for (volatile long delay = 0; delay < 10000; delay++) {
	}

	/* Free allocated memory */
	free(altitude_digits);
	free(bits_units);
	free(bits_tens);
	free(bits_hundreds);
}

void display_roll() {
	//TODO: Display roll
}

/* *************************************************** */
/* ******************** UTILITIES ******************** */
/* *************************************************** */

/**
 * Constrains a float value within the given range.
 *
 * @param x The input value.
 * @param min_val Minimum allowable value.
 * @param max_val Maximum allowable value.
 * @return Clamped value within [min_val, max_val].
 */
float clamp(const float x, const float min_val, const float max_val) {
	return (x < min_val) ? min_val : (x > max_val) ? max_val : x;
}

/**
 * Splits an integer number into an array of its decimal digits (the least significant first).
 *
 * @param number The input integer number.
 * @param n_digits Number of digits to extract.
 * @return Dynamically allocated array of digits, or NULL if allocation fails.
 */
int *extract_digits(int number, const int n_digits) {
	int *digits = (int *) malloc(n_digits * sizeof(int));
	/* Safety check */
	if (digits == NULL) {
		return NULL;
	}

	for (int i = 0; i < n_digits; i++) {
		digits[i] = number % 10;
		number /= 10;
	}
	return digits;
}

/**
 * Converts a decimal number to its binary representation as an array of bits.
 *
 * @param decimal Input decimal number.
 * @param n_bits Number of bits in the result.
 * @return Dynamically allocated bit array (the most significant bit first).
 */
int *decimal_to_binary(const int decimal, const int n_bits) {
	int *bits = (int *) malloc(n_bits * sizeof(int));
	for (int i = n_bits - 1; i >= 0; i--) {
		bits[n_bits - 1 - i] = (decimal >> i) & 1;
	}
	return bits;
}

/**
 * Sends a sequence of bits to a digital display using PORTD and specific channels.
 *
 * @param bits Array of bits to send.
 * @param n_bits Number of bits to send.
 * @param channel_d Data pin index on PORTD.
 * @param channel_clk Clock pin index on PORTD.
 */
void send_bits_to_display(const int *bits, const int n_bits, const unsigned int channel_d, const unsigned int channel_clk) {
	/* Clear ports before setting new bits */
	PORTD &= ~((1 << channel_d) | (1 << channel_clk));
	/* Send bits to port */
	for (int i = n_bits - 1; i >= 0; i--) {
		if (bits[i]) {
			PORTD |= (1 << channel_d);
		}
		else {
			PORTD &= ~(1 << channel_d);
		}

		/* Clock pulse */
		PORTD |= (1 << channel_clk);
		PORTD &= ~(1 << channel_clk);
	}
}

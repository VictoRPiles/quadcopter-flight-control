int current_altitude;
int altitude_lower_limit;
int altitude_upper_limit;

int current_roll_angle;
int roll_angle_min;
int roll_angle_max;

const int reload_value = get_reload_value(20000, 16, 8); /* For 20ms, 16us and pre-scaler 8 */
long motor_angle_left = 0;
long motor_angle_right = 0;

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
	DDRB = 0b00000110; /* Set [PB0, PB3-PB7] as input and [PB1, PB2] as output*/
	DDRC = 0b00000000; /* Set [PC0-PC7] as input */
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
	/* Configure timer 1 */
	TCCR1A = 0b00000000; /* Normal operation of the I/O lines */
	TCCR1B = 0b00000010; /* Pre-scalar of 8 */
	TCNT1H = reload_value / 256; /* Time interval for 20ms */
	TCNT1L = reload_value % 256; /* Initial value of the timer */
	TIMSK1 = (1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B); /* Timer 1 overflow and compare interrupt enabled */
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

/**
 * Compute the reload value for a specified configuration.
 *
 * @param time_interval Time interval in milliseconds.
 * @param frequency_io Frequency of the microcontroller in milliseconds.
 * @param pre_scaler Pre-scaler value.
 * @return The appropriate reload value.
 */
int get_reload_value(const int time_interval, const int frequency_io, const int pre_scaler) {
	return 65536 - (time_interval * frequency_io) / pre_scaler;
}

ISR(TIMER1_OVF_vect) {
	TCNT1 = reload_value; // Restart 20ms timer
	PORTB |= (1 << 2); // Set PB2 HIGH (start of pulse)

	// FIXME: Motor angle is 11.2º greater
	motor_angle_left = motor_angle_left - 11;
	motor_angle_right = motor_angle_right - 11;
	// Set Compare Match A to clear pin after `pulse_width_us`
	// pulse_width_us * 2 = number of ticks (because each tick = 0.5us)
	const long pulse_width_left = angle_to_pulse(motor_angle_left, 0, 180, 1000, 2000);
	OCR1A = TCNT1 + (pulse_width_left * 2);

	PORTB |= (1 << 1); // Set PB1 HIGH (start of pulse)

	// Setup Compare Match B for RIGHT MOTOR
	const long pulse_width_right = angle_to_pulse(motor_angle_right, 0, 180, 1000, 2000);
	OCR1B = TCNT1 + (pulse_width_right * 2);
}

ISR(TIMER1_COMPA_vect) {
	PORTB &= ~(1 << 2); // Set PB2 LOW (end of pulse)
}

ISR(TIMER1_COMPB_vect) {
	PORTB &= ~(1 << 1); // Set PB1 LOW (end of pulse)
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

void control_motors(const float speed_altitude, const float speed_roll) {
	/* Roll deviation from center */
	const float roll_effect = (speed_roll - 50.0f) * 2;

	/* Weighted motor speed: 75% altitude, 25% roll */
	const float total_speed_left = 0.75f * speed_altitude - 0.25f * roll_effect;
	const float total_speed_right = 0.75f * speed_altitude + 0.25f * roll_effect;

	/* Map to angles [0, 180] */
	const float angle_left = (total_speed_left / 100.0f) * 180.0f;
	const float angle_right = (total_speed_right / 100.0f) * 180.0f;

	/* Set angle to motors */
	motor_angle_left = (long) angle_left;
	motor_angle_right = (long) angle_right;
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

long angle_to_pulse(const long x, const long in_min, const long in_max, const long out_min, const long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

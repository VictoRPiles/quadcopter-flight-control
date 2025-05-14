int altitude;
int altitude_min;
int altitude_max;

int roll;
int roll_min;
int roll_max;

/**
 * Read the analog value from a potentiometer connected to the specified channel
 *
 * @param channel Channel to read from
 * @return The analog value of the potentiometer
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
 * Scale an ADC value to a percentage
 *
 * @param value Analog value
 * @return Analog value scaled to percentage
 */
float scale_adc_to_percent(const float value) { return ((float) value / 1023.0f) * 100.0f; }

/**
 * Constrain the resolution of a value within a range
 *
 * @param x Value
 * @param min_val Lower limit
 * @param max_val Upper limit
 * @return A value inside the limits
 */
float clamp(const float x, const float min_val, const float max_val) {
	return (x < min_val) ? min_val : (x > max_val) ? max_val : x;
}

static int fractional_part = 0;
const int SCALE = 100;

/**
 * Determines the climb rate depending on the altitude potentiometer and calculates the new altitude
 *
 * @param pot_altitude_percentage The level of the altitude potentiometer
 */
void control_altitude(const int pot_altitude_percentage) {
	/* Clamp input to [0, 100] */
	const int percent = (int) clamp((float) pot_altitude_percentage, 0, 100);
	/* Calculate climb with scaling */
	fractional_part += percent - 50; // Range -50 to +50

	/* When accumulated enough for at least 1 unit */
	const int climb = fractional_part / SCALE;
	fractional_part %= SCALE;

	altitude += climb;
	/* Clamp altitude */
	altitude = (int) clamp((float) altitude, (float) altitude_min, (float) altitude_max);
}

/**
 * Calculates the new roll angle
 *
 * @param pot_roll_percentage The level of the roll potentiometer
 */
void control_roll(const int pot_roll_percentage) {
	/* Clamp input to [0, 100] */
	const float percent = clamp((float) pot_roll_percentage, 0, 100);

	/* Linearly map percentage to roll */
	roll = (int) ((float) roll_min + ((float) (roll_max - roll_min) * percent) / 100.0f);
}

// TODO: Parameters (refresh rate, digits, altitude...)
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
	for (volatile long delay = 0; delay < 10000; delay++) {
	}

	/* Free allocated memory */
	free(altitude_digits);
	free(bits_units);
	free(bits_tens);
	free(bits_hundreds);
}

/**
 * Split a number in digits and return as an array
 *
 * @param number Integer number
 * @param n_digits Number of digits to extract
 * @return Array with the digits
 */
int *digits(int number, const int n_digits) {
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
 * Send an array of bits and clock pulse over PIND
 *
 * @param bits Data to be sent
 * @param n_bits Number of bits sent
 * @param channel_d Data channel of the display
 * @param channel_clk Clock channel of the display
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
 * Extract the bits of a decimal number to an array
 *
 * @param decimal A decimal number
 * @param n_bits Number of bits to extract
 * @return An array with the bits
 */
int *decToBin(const int decimal, const int n_bits) {
	int *bits = (int *) malloc(n_bits * sizeof(int));
	for (int i = n_bits - 1; i >= 0; i--) {
		bits[n_bits - 1 - i] = (decimal >> i) & 1;
	}
	return bits;
}

/**
 * Configures the board and initialises the setup variables
 */
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

	roll_min = -45;
	roll_max = 45;
	roll = roll_min;
}

/**
 * Main execution loop
 */
void loop() {
	/* If the controller is ON (PB0 high level) */
	if ((PINB & (1 << 0))) {
		const float pot_altitude = scale_adc_to_percent(read_potentiometer(0));
		const float pot_roll = scale_adc_to_percent(read_potentiometer(1));

		const float speed_altitude = 0.75f * pot_altitude;
		const float speed_roll = 0.25f * pot_roll;

		control_altitude((int) pot_altitude);
		control_roll((int) pot_roll);
		display_altitude();
	}
}

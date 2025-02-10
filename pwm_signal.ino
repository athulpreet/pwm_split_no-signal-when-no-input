#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

volatile uint16_t last_capture = 0;
volatile uint16_t period = 0;
volatile uint8_t new_capture = 0;
volatile uint16_t frequency = 0;
volatile uint8_t signal_present = 0;
volatile uint16_t overflow_count = 0;
#define SIGNAL_TIMEOUT 3  // Number of Timer1 overflows to wait before declaring signal loss

// UART initialization
void uart_init() {
    uint16_t ubrr = 103;
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = (1<<URSEL0)|(0<<UMSEL0)|(0<<UPM01)|(0<<UPM00)|(0<<USBS0)|(3<<UCSZ00);
}

void uart_transmit(unsigned char data) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
    while (!(UCSR0A & (1<<TXC0)));
    UCSR0A |= (1<<TXC0);
}

void uart_string(char *str) {
    while(*str) {
        uart_transmit(*str);
        str++;
    }
}

// Function to update PWM frequency
void update_pwm_frequency(uint16_t freq) {
    if(freq < 10 || freq > 20000) return; // Safety bounds
    
    // Calculate ICR3 value: F_CPU/(prescaler * F_desired) - 1
    uint16_t icr_value = (F_CPU/(8UL * freq)) - 1;
    
    // Update Timer3 settings
    ICR3 = icr_value;
    OCR3A = ICR3/2;  // Maintain 50% duty cycle
    OCR3B = ICR3/2;
}

// Timer1 Overflow Interrupt Service Routine
ISR(TIMER1_OVF_vect) {
    overflow_count++;
    if(overflow_count >= SIGNAL_TIMEOUT) {
        signal_present = 0;
        frequency = 0;
        // Stop PWM output when no signal
        OCR3A = 0;
        OCR3B = 0;
    }
}

// Timer1 Input Capture Interrupt Service Routine
ISR(TIMER1_CAPT_vect) 
{
    overflow_count = 0;  // Reset timeout counter
    signal_present = 1;  // Signal is present
    
    uint16_t current_capture = ICR1;
    
    if (current_capture >= last_capture) {
        period = current_capture - last_capture;
    } else {
        period = (0xFFFF - last_capture) + current_capture;
    }
    
    last_capture = current_capture;
    new_capture = 1;
    
    if(period != 0) {
        frequency = F_CPU/(8UL * period);
        // Update PWM frequency directly in the ISR
        update_pwm_frequency(frequency);
    }
}

int main(void) 
{
    char buffer[50];
    
    // Initialize UART
    uart_init();
    
    // Configure Timer1 for Input Capture
    TCCR1A = 0;  // Normal mode
    TCCR1B = (1 << ICNC1) |    // Enable noise canceler
             (1 << ICES1) |     // Capture on rising edge
             (1 << CS11);       // Prescaler = 8
             
    TCNT1 = 0;
    
    // Configure ICP1 (PE0) as input with pull-up
    DDRE &= ~(1 << PE0);
    PORTE |= (1 << PE0);
    
    // Set up Timer3 for PWM output
    TCCR3A = (1 << COM3A1) |   // Non-inverting mode channel A
             (1 << COM3B1) |    // Non-inverting mode channel B
             (1 << WGM31);      // Fast PWM mode
             
    TCCR3B = (1 << WGM33) | (1 << WGM32) | // Fast PWM mode
             (1 << CS31);       // Prescaler = 8
    
    // Configure PWM output pins
    DDRD |= (1 << PD4);  // OC3A on PD4
    DDRB |= (1 << PB4);  // OC3B on PB4
    
    // Enable Timer1 input capture and overflow interrupts
    TIMSK |= (1 << TICIE1) | (1 << TOIE1);
    
    // Enable global interrupts
    sei();

    uart_string("Frequency Matching System Started\r\n");

    while(1) 
    {
        if(new_capture || !signal_present)  // Update display when there's new data or signal loss
        {
            if(signal_present) {
                sprintf(buffer, "Frequency: %u Hz\r\n", frequency);
            } else {
                sprintf(buffer, "No Signal Detected\r\n");
            }
            uart_string(buffer);
            new_capture = 0;
            _delay_ms(500);
        }
    }
    
    return 0;
}

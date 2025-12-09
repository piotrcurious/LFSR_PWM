/*
  arduino_lfsr_pwm_examples_optimized.ino

  Optimized ISR variant for LFSR-based pseudo-random PWM on AVR (ATmega328P).
  This version implements the critical ISR as a NAKED interrupt written in
  inline assembly which saves only the registers it actually needs instead of
  the full compiler-generated prologue/epilogue. The goal is to minimize cycle
  overhead per tick while remaining functionally equivalent to the earlier
  C-based ISR.

  NOTES & TRADEOFFS
  - This sketch is AVR-specific (ATmega328P / Arduino UNO family). Do not use
    the naked ISR on other architectures without porting the assembly.
  - To keep the ISR small we use 16-bit counters (tick_count, high_count).
    If your measurement window requires >65535 ticks you must change them to
    32-bit and accept higher ISR cost for multi-byte arithmetic or perform the
    aggregation in the main loop.
  - Because the ISR is tiny it does *not* perform the measurement-window
    comparison; the main loop checks tick_count and resets counters with
    interrupts temporarily disabled. That keeps the ISR fast.
  - Direct port manipulation (IN/OUT/ORI/ANDI) is used to set the output pin
    atomically and cheaply.
  - You must ensure the LFSR state variables are placed in SRAM and labeled as
    volatile so the assembler addresses are correct.

  Assembly logic (high level):
    - save SREG + minimal set of registers we use
    - load 16-bit lfsr_state
    - capture LSB (old low bit)
    - shift the 16-bit state right by 1
    - if captured bit was 1 -> XOR with 16-bit galois mask
    - store new state
    - update PORTB bit for OUT_PIN (using PORTB read-modify-write)
    - increment 16-bit tick_count; if output was high increment high_count
    - restore minimal registers and SREG, return with RETI

  Build: compile as normal with Arduino IDE or avr-gcc. The assembly uses
  standard ATMega328P instructions and I/O register names (PORTB).
*/

#include <Arduino.h>
#include <avr/io.h>

// Configuration
#define OUT_PIN          9       // digital pin 9 -> PORTB bit 1
#define OUT_PIN_BIT      1
#define TICK_HZ          31250UL // timer tick frequency
#define MEAS_TICKS       3125    // measurement window (must be < 65536 for uint16_t)

// Minimal data structures (placed in SRAM)
volatile uint16_t lfsr_state = 1;            // current 16-bit LFSR state (non-zero)
volatile uint16_t lfsr_mask_current = 0xb400; // Galois feedback mask (16-bit)
volatile uint16_t tick_count = 0;            // 16-bit tick counter (ISR increments)
volatile uint16_t high_count = 0;            // 16-bit count of high outputs

// runtime control (set from main loop)
volatile uint8_t active_mode = 2; // 2 == LFSR mode (kept for API compatibility)
volatile bool use_galois = true;  // unused here, kept for compatibility

// Forward prototypes
void setup_timer1_ctc(uint32_t tick_hz);
void apply_pwm_level(uint8_t level);

// ---------- Naked ISR optimized in assembly ----------
// The ISR saves only registers it actually uses: r0 (temp for SREG), r18, r19,
// r20, r21 (16-bit temp for counters), r22, r23 (mask), r24, r25 (state).
// This is much smaller than the compiler's full save of r0..r31.

ISR(TIMER1_COMPA_vect, ISR_NAKED) {
  asm volatile (
    // --- prologue: save SREG and used registers ---
\
    "in r0, __SREG__            
\"  // r0 := SREG
\
    "push r0                    
\"  // save SREG on stack
\
    "push r18                   
\"  // will use r18 as small flag
\
    "push r19                   
\"  // temp for PORTB RMW
\
    "push r20                   
\"  // counter low byte temp
\
    "push r21                   
\"  // counter high byte temp
\
    "push r22                   
\"  // mask low byte
\
    "push r23                   
\"  // mask high byte
\
    "push r24                   
\"  // state low byte
\
    "push r25                   
\"  // state high byte
\

    // --- load lfsr_state (16-bit little endian) into r24 (low), r25 (high) ---
\
    "lds r24, lfsr_state        
\"  // load low byte
\
    "lds r25, lfsr_state+1      
\"  // load high byte
\

    // --- capture original LSB into r18 (0 or 1) ---
\
    "clr r18                    
\"  // r18 = 0
\
    "sbrc r24, 0                
\"  // skip if bit0 is clear
\
    "ldi r18, 1                 
\"  // r18 = 1 if original LSB was 1
\

    // --- shift 16-bit right by 1: (low >>1), high rotates in from low's bit7 ---
\
    "lsr r24                    
\"  // low >>= 1, C <= old low bit0
\
    "ror r25                    
\"  // high >>=1 with carry from old low bit0
\

    // --- load current mask into r22:r23 ---
\
    "lds r22, lfsr_mask_current 
\"  // mask low
\
    "lds r23, lfsr_mask_current+1
\"  // mask high
\

    // --- if captured bit was 1 then XOR state with mask ---
\
    "tst r18                    
\"  // set Z flag if r18==0
\
    "breq 1f                    
\"  // if r18==0 skip XOR
\
    "eor r24, r22               
\"  // state_low ^= mask_low
\
    "eor r25, r23               
\"  // state_high ^= mask_high
\
    "1:                         
\"  // continue

    // --- store new state back to SRAM ---
\
    "sts lfsr_state, r24       
\"  // write low
\
    "sts lfsr_state+1, r25     
\"  // write high
\

    // --- update PORTB.OUT bit according to new state LSB ---
\
    "in r19, _SFR_IO_ADDR(PORTB)
\"  // read PORTB (I/O space)
    "andi r19, %[pinmask_clear] 
\"  // clear the output bit in the temp reg
    "sbrc r24, 0                
\"  // if (state & 1) != 0 execute next instruction
    "ori r19, %[pinmask_set]    
\"  // set the bit in temp reg if output bit is 1
    "out _SFR_IO_ADDR(PORTB), r19
\"  // write back PORTB

    // --- increment tick_count (16-bit) ---
\
    "lds r20, tick_count       
\"  // low
    "lds r21, tick_count+1     
\"  // high
    "inc r20                   
\"  // low++
    "brne 2f                   
\"  // if low != 0 after inc skip inc high
    "inc r21                   
\"  // carry to high
    "2:                        
\"  // continue
    "sts tick_count, r20       
\"  // store low
    "sts tick_count+1, r21     
\"  // store high

    // --- if output is high (r24 bit0) increment high_count ---
\
    "sbrc r24, 0               
\"  // skip next if bit0 is clear (i.e. if output low skip increment)
    "rjmp 3f                   
\"  // if bit set, jump to increment
    "rjmp 4f                   
\"  // else jump past increment
    "3:                        
\"  // increment high_count
    "lds r20, high_count       
\"  // low
    "lds r21, high_count+1     
\"  // high
    "inc r20                   
\"  // low++
    "brne 5f                   
\"  // if low != 0 after inc skip inc high
    "inc r21                   
\"  // carry to high
    "5:                        
\"  // continue
    "sts high_count, r20       
\"  // store low
    "sts high_count+1, r21     
\"  // store high
    "4:                        
\"  // continue (no-op)

    // --- epilogue: restore registers and SREG, return with RETI ---
\
    "pop r25                    
\"  // restore state high
    "pop r24                    
\"  // restore state low
    "pop r23                    
\"  // restore mask high
    "pop r22                    
\"  // restore mask low
    "pop r21                    
\"  // restore counter high temp
    "pop r20                    
\"  // restore counter low temp
    "pop r19                    
\"  // restore port temp
    "pop r18                    
\"  // restore flag
    "pop r0                     
\"  // pop saved SREG into r0
    "out __SREG__, r0           
\"  // restore SREG
    "reti                      
\"

    : // no outputs
    : [pinmask_clear] "I" (~(1 << OUT_PIN_BIT) & 0xFF),
      [pinmask_set]   "I" ( (1 << OUT_PIN_BIT) & 0xFF)
    : "r0" // tell compiler we clobber r0 (already pushed/popped)
  );
}

// ---------- Timer setup (same as before) ----------
void setup_timer1_ctc(uint32_t tick_hz) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  const uint32_t F_CPU_HZ = F_CPU;
  uint16_t prescaler = 1;
  uint16_t csbits = _BV(CS10);
  uint32_t ocr = (F_CPU_HZ / (prescaler * tick_hz)) - 1;
  if (ocr > 0xFFFF) {
    prescaler = 8;
    csbits = _BV(CS11);
    ocr = (F_CPU_HZ / (prescaler * tick_hz)) - 1;
  }
  if (ocr > 0xFFFF) ocr = 65535;
  OCR1A = (uint16_t)ocr;
  TCCR1B |= _BV(WGM12);
  TCCR1B |= csbits;
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}

// ---------- Helpers ----------
void apply_pwm_level(uint8_t level) {
  // In a production system read a 256-entry PROGMEM LUT and choose mask & seed.
  // Here we simply choose a demo mask and ensure non-zero state.
  if (level == 0) {
    lfsr_mask_current = 0x0001; // placeholder (not a real LFSR)
  } else if (level >= 255) {
    lfsr_mask_current = 0xFFFF; // placeholder
  } else {
    lfsr_mask_current = 0xB400; // sample primitive polynomial -> convert to Galois form offline
  }
  if (lfsr_state == 0) lfsr_state = 1;
}

// ---------- Setup & main loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  // configure pin as output - using direct DDRB
  DDRB |= (1 << OUT_PIN_BIT);
  PORTB &= ~(1 << OUT_PIN_BIT);

  apply_pwm_level(128);
  setup_timer1_ctc(TICK_HZ);
  Serial.println("Optimized LFSR PWM (naked ISR) ready");
}

void loop() {
  static uint32_t next_change_ms = 0;
  static uint8_t level = 0;
  uint32_t now = millis();

  if (now >= next_change_ms) {
    level += 32;
    apply_pwm_level(level);
    next_change_ms = now + 2000;
    Serial.print("Set level = "); Serial.println(level);
  }

  // Poll measurement counters and print when we collected MEAS_TICKS
  // Disable interrupts briefly to read 16-bit counters atomically
  noInterrupts();
  uint16_t t = tick_count;
  uint16_t h = high_count;
  if (t >= MEAS_TICKS) {
    tick_count = 0;
    high_count = 0;
    interrupts();
    float achieved = ((float)h) / ((float)t);
    Serial.print("Achieved="); Serial.print(achieved * 100.0f, 2);
    Serial.print("% ticks="); Serial.print(t);
    Serial.print(" high="); Serial.println(h);
  } else {
    interrupts();
  }

  // non-blocking serial handling
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      use_galois = !use_galois;
      Serial.print("use_galois toggled: "); Serial.println(use_galois);
    }
  }
  delay(10);
}

/*
  Final notes:
  - The naked ISR approach reduces cycles by saving/restoring only a handful of
    registers. Measured cycle savings depend on your previous ISR body but will
    typically be substantial compared to the default full-register save.
  - Keep the ISR body very small: offload heavier work (comparisons, prints,
    long arithmetic) to the main loop. Use atomic reads (noInterrupts/interrupts)
    in the main loop when accessing multi-byte variables modified in the ISR.
  - If you want to further micro-optimize, you can tune which registers are
    actually pushed and omitted -- but be very careful: removing a register
    that the assembler or C runtime expects preserved will break the program.
*/

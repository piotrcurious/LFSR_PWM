/*
  arduino_lfsr_pwm_multi_optimized.ino

  Multi-channel LFSR-based pseudo-random PWM for AVR (ATmega328P / Arduino UNO).
  - Supports up to 6 simultaneous PWM outputs located on PORTB (digital pins 8..13)
    to allow very cheap atomic updates (single PORTB read-modify-write).
  - ISR implemented as a naked interrupt in inline assembly and fully unrolled
    across channels for minimal per-channel overhead.
  - Only LFSR-based PWM is retained; all other PWM methods removed.

  Limitations & rationale
  - To keep the ISR minimal and simple, all channels are constrained to PORTB
    (pins 8..13 on Arduino UNO, which map to PORTB bits 0..5). This allows a
    single RMW of PORTB per tick and avoids needing cross-port coordination.
  - Counters are 16-bit (tick_count[] and high_count[]). If you need windows
    larger than 65535 ticks, you should switch to 32-bit counters (costs extra
    ISR cycles) or aggregate in the main loop.
  - The ISR body is unrolled for NUM_CHANNELS to avoid indexed memory math and
    branching. For small NUM_CHANNELS (<=6) this gives excellent performance.

  How it works
  - Each channel has a 16-bit LFSR state and a 16-bit Galois mask stored in SRAM.
  - On each timer tick the ISR does, for each channel:
      * load state
      * compute one Galois step (shift right, conditional XOR with mask)
      * store updated state
      * collect the new LSB (output bit) and set/clear the corresponding PORTB
        temporary mask accordingly
      * increment per-channel tick counter and high counter if output==1
  - After all channels processed, write the computed PORTB value once and
    return.

  Usage
  - Define NUM_CHANNELS (1..6) and adjust channel_mask_bits[] to pick which
    PORTB bit each channel occupies (default maps channels 0..NUM-1 to bits
    0..NUM-1 corresponding to Arduino pins 8..13).
  - Replace demo masks with your generated PROGMEM LUT and implement per-level
    selection as needed.

  Build: Arduino IDE or avr-gcc. Make sure code compiled for ATmega328P.
*/

#include <Arduino.h>
#include <avr/io.h>

// -------------------- Configuration --------------------
#define NUM_CHANNELS 4 // set 1..6
// Channels are placed on PORTB bits (Arduino digital pins 8..13 map to PORTB0..5)
// channel_mask_bits[i] picks the PORTB bit for channel i (0..5)
const uint8_t channel_mask_bits[NUM_CHANNELS] = {0, 1, 2, 3};

#define TICK_HZ    31250UL
#define MEAS_TICKS 3125    // measurement window (must be < 65536)

// -------------------- Per-channel storage (SRAM) --------------------
volatile uint16_t lfsr_state[NUM_CHANNELS] = {1,1,1,1,1,1};
volatile uint16_t lfsr_mask_current[NUM_CHANNELS] = {0xB400,0xB400,0xB400,0xB400,0xB400,0xB400};
volatile uint16_t tick_count[NUM_CHANNELS] = {0};
volatile uint16_t high_count[NUM_CHANNELS] = {0};

// runtime control
volatile uint8_t active_channels = NUM_CHANNELS; // number of active channels at runtime (<= NUM_CHANNELS)

// Forward declarations
void setup_timer1_ctc(uint32_t tick_hz);
void apply_pwm_level_channel(uint8_t ch, uint8_t level);

// -------------------- Naked ISR (unrolled multi-channel LFSR) --------------------
// Saves only registers used; unrolled for NUM_CHANNELS (1..6). Works only when
// channels are on PORTB so we can batch updates with a single PORTB write.
ISR(TIMER1_COMPA_vect, ISR_NAKED) {
  asm volatile (
    // --- prologue: save SREG and minimal registers used ---
\
    "in r0, __SREG__            
\"  // r0 := SREG
\
    "push r0                    
\"  // save SREG
\
    "push r18                   
\"  // temp/flag
\
    "push r19                   
\"  // PORTB temp low byte
\
    "push r20                   
\"  // tmp low
\
    "push r21                   
\"  // tmp high
\
    "push r22                   
\"  // mask low
\
    "push r23                   
\"  // mask high
\
    "push r24                   
\"  // state low
\
    "push r25                   
\"  // state high
\

    // --- prepare a PORTB temp variable in r19; read current PORTB and clear bits for channels ---
\
    "in r19, _SFR_IO_ADDR(PORTB)
\"  // read PORTB
\
    "ldi r18, 0xFF              
\"  // r18 = 0xFF (will be ANDed with clear mask later)
\"
    // build clear mask by zeroing channel bits: do successive ANDI operations using compile-time constants

    : /* outputs */
    : /* inputs */
    : "r0" /*clobber r0 noted*/
  );

  // Because writing fully unrolled assembly with compile-time channel masks is verbose
  // we will provide specialized unrolled sequences depending on NUM_CHANNELS using preprocessor.

#if NUM_CHANNELS >= 1
  asm volatile (
    // --- Channel 0 ---
\
    "lds r24, lfsr_state        
\"  // ch0 low
\
    "lds r25, lfsr_state+1      
\"  // ch0 high
\
    "clr r18                    
\"  // r18=0
\
    "sbrc r24, 0                
\"  // skip if bit0 clear
\
    "ldi r18, 1                 
\"  // r18=1 if original LSB was 1
\
    "lsr r24                    
\"  // shift low
\
    "ror r25                    
\"  // rotate high
\
    "lds r22, lfsr_mask_current 
\"  // mask low
    "lds r23, lfsr_mask_current+1
\"  // mask high
    "tst r18                    
\"  // test if need xor
\
    "breq 1f                    
\"  // skip xor if r18==0
\
    "eor r24, r22               
\"  // xor low
\
    "eor r25, r23               
\"  // xor high
\
    "1:                         
\"
    "sts lfsr_state, r24        
\"  // store low
\
    "sts lfsr_state+1, r25      
\"  // store high
\"
    // evaluate new LSB into r18 (0/1)
    "andi r24, 0x01             
\"  // keep LSB in r24 (others cleared)
\
    "mov r18, r24               
\"  // r18 := output bit (0/1)
\
    // set/clear PORTB temp bit for channel 0 (bit number from channel_mask_bits[0] constant)
    "andi r19, %[ch0_clear]     
\"  // clear ch0 bit in temp
\
    "sbrc r18, 0                
\"  // if output==1 skip next
\
    "rjmp 2f                    
\"  // if output==0 skip set
\
    "ori r19, %[ch0_set]        
\"  // set ch0 bit if output==1
\
    "2:                         
\"
    // increment tick_count[0]
    "lds r20, tick_count        
\"  // low
    "lds r21, tick_count+1      
\"  // high
    "inc r20                    
\"  // low++
    "brne 3f                    
\"  // if no carry
    "inc r21                    
\"  // carry
    "3:                         
\"
    "sts tick_count, r20        
\"
    "sts tick_count+1, r21      
\"
    // if output was 1 increment high_count[0]
    "sbrc r18, 0                
\"  // if r18==0 skip next
    "rjmp 4f                    
\"
    "rjmp 5f                    
\"
    "4:                         
\"
    "lds r20, high_count        
\"
    "lds r21, high_count+1      
\"
    "inc r20                    
\"
    "brne 6f                    
\"
    "inc r21                    
\"
    "6:                         
\"
    "sts high_count, r20        
\"
    "sts high_count+1, r21      
\"
    "5:                         
\"
    : /* outputs */
    : [ch0_clear] "I" (~(1 << 0) & 0xFF), [ch0_set] "I" (1 << 0)
    : "r18","r19","r20","r21","r22","r23","r24","r25","r0"
  );
#endif

#if NUM_CHANNELS >= 2
  asm volatile (
    // --- Channel 1 ---
\
    "lds r24, lfsr_state+2      
\"  // ch1 low
\
    "lds r25, lfsr_state+3      
\"  // ch1 high
\
    "clr r18                    
\"
    "sbrc r24, 0                
\"
    "ldi r18, 1                 
\"
    "lsr r24                    
\"
    "ror r25                    
\"
    "lds r22, lfsr_mask_current+2
\"
    "lds r23, lfsr_mask_current+3
\"
    "tst r18                    
\"
    "breq 1f                    
\"
    "eor r24, r22               
\"
    "eor r25, r23               
\"
    "1:                         
\"
    "sts lfsr_state+2, r24      
\"
    "sts lfsr_state+3, r25      
\"
    "andi r24, 0x01             
\"
    "mov r18, r24               
\"
    "andi r19, %[ch1_clear]     
\"
    "sbrc r18, 0                
\"
    "rjmp 2f                    
\"
    "ori r19, %[ch1_set]        
\"
    "2:                         
\"
    "lds r20, tick_count+2      
\"
    "lds r21, tick_count+3      
\"
    "inc r20                    
\"
    "brne 3f                    
\"
    "inc r21                    
\"
    "3:                         
\"
    "sts tick_count+2, r20      
\"
    "sts tick_count+3, r21      
\"
    "sbrc r18, 0                
\"
    "rjmp 4f                    
\"
    "rjmp 5f                    
\"
    "4:                         
\"
    "lds r20, high_count+2      
\"
    "lds r21, high_count+3      
\"
    "inc r20                    
\"
    "brne 6f                    
\"
    "inc r21                    
\"
    "6:                         
\"
    "sts high_count+2, r20      
\"
    "sts high_count+3, r21      
\"
    "5:                         
\"
    :
    : [ch1_clear] "I" (~(1 << 1) & 0xFF), [ch1_set] "I" (1 << 1)
    : "r18","r19","r20","r21","r22","r23","r24","r25"
  );
#endif

#if NUM_CHANNELS >= 3
  asm volatile (
    // --- Channel 2 ---
\
    "lds r24, lfsr_state+4      
\"
    "lds r25, lfsr_state+5      
\"
    "clr r18                    
\"
    "sbrc r24, 0                
\"
    "ldi r18, 1                 
\"
    "lsr r24                    
\"
    "ror r25                    
\"
    "lds r22, lfsr_mask_current+4
\"
    "lds r23, lfsr_mask_current+5
\"
    "tst r18                    
\"
    "breq 1f                    
\"
    "eor r24, r22               
\"
    "eor r25, r23               
\"
    "1:                         
\"
    "sts lfsr_state+4, r24      
\"
    "sts lfsr_state+5, r25      
\"
    "andi r24, 0x01             
\"
    "mov r18, r24               
\"
    "andi r19, %[ch2_clear]     
\"
    "sbrc r18, 0                
\"
    "rjmp 2f                    
\"
    "ori r19, %[ch2_set]        
\"
    "2:                         
\"
    "lds r20, tick_count+4      
\"
    "lds r21, tick_count+5      
\"
    "inc r20                    
\"
    "brne 3f                    
\"
    "inc r21                    
\"
    "3:                         
\"
    "sts tick_count+4, r20      
\"
    "sts tick_count+5, r21      
\"
    "sbrc r18, 0                
\"
    "rjmp 4f                    
\"
    "rjmp 5f                    
\"
    "4:                         
\"
    "lds r20, high_count+4      
\"
    "lds r21, high_count+5      
\"
    "inc r20                    
\"
    "brne 6f                    
\"
    "inc r21                    
\"
    "6:                         
\"
    "sts high_count+4, r20      
\"
    "sts high_count+5, r21      
\"
    "5:                         
\"
    :
    : [ch2_clear] "I" (~(1 << 2) & 0xFF), [ch2_set] "I" (1 << 2)
    : "r18","r19","r20","r21","r22","r23","r24","r25"
  );
#endif

#if NUM_CHANNELS >= 4
  asm volatile (
    // --- Channel 3 ---
\
    "lds r24, lfsr_state+6      
\"
    "lds r25, lfsr_state+7      
\"
    "clr r18                    
\"
    "sbrc r24, 0                
\"
    "ldi r18, 1                 
\"
    "lsr r24                    
\"
    "ror r25                    
\"
    "lds r22, lfsr_mask_current+6
\"
    "lds r23, lfsr_mask_current+7
\"
    "tst r18                    
\"
    "breq 1f                    
\"
    "eor r24, r22               
\"
    "eor r25, r23               
\"
    "1:                         
\"
    "sts lfsr_state+6, r24      
\"
    "sts lfsr_state+7, r25      
\"
    "andi r24, 0x01             
\"
    "mov r18, r24               
\"
    "andi r19, %[ch3_clear]     
\"
    "sbrc r18, 0                
\"
    "rjmp 2f                    
\"
    "ori r19, %[ch3_set]        
\"
    "2:                         
\"
    "lds r20, tick_count+6      
\"
    "lds r21, tick_count+7      
\"
    "inc r20                    
\"
    "brne 3f                    
\"
    "inc r21                    
\"
    "3:                         
\"
    "sts tick_count+6, r20      
\"
    "sts tick_count+7, r21      
\"
    "sbrc r18, 0                
\"
    "rjmp 4f                    
\"
    "rjmp 5f                    
\"
    "4:                         
\"
    "lds r20, high_count+6      
\"
    "lds r21, high_count+7      
\"
    "inc r20                    
\"
    "brne 6f                    
\"
    "inc r21                    
\"
    "6:                         
\"
    "sts high_count+6, r20      
\"
    "sts high_count+7, r21      
\"
    "5:                         
\"
    :
    : [ch3_clear] "I" (~(1 << 3) & 0xFF), [ch3_set] "I" (1 << 3)
    : "r18","r19","r20","r21","r22","r23","r24","r25"
  );
#endif

#if NUM_CHANNELS >= 5
  asm volatile (
    // --- Channel 4 ---
\
    "lds r24, lfsr_state+8      
\"
    "lds r25, lfsr_state+9      
\"
    "clr r18                    
\"
    "sbrc r24, 0                
\"
    "ldi r18, 1                 
\"
    "lsr r24                    
\"
    "ror r25                    
\"
    "lds r22, lfsr_mask_current+8
\"
    "lds r23, lfsr_mask_current+9
\"
    "tst r18                    
\"
    "breq 1f                    
\"
    "eor r24, r22               
\"
    "eor r25, r23               
\"
    "1:                         
\"
    "sts lfsr_state+8, r24      
\"
    "sts lfsr_state+9, r25      
\"
    "andi r24, 0x01             
\"
    "mov r18, r24               
\"
    "andi r19, %[ch4_clear]     
\"
    "sbrc r18, 0                
\"
    "rjmp 2f                    
\"
    "ori r19, %[ch4_set]        
\"
    "2:                         
\"
    "lds r20, tick_count+8      
\"
    "lds r21, tick_count+9      
\"
    "inc r20                    
\"
    "brne 3f                    
\"
    "inc r21                    
\"
    "3:                         
\"
    "sts tick_count+8, r20      
\"
    "sts tick_count+9, r21      
\"
    "sbrc r18, 0                
\"
    "rjmp 4f                    
\"
    "rjmp 5f                    
\"
    "4:                         
\"
    "lds r20, high_count+8      
\"
    "lds r21, high_count+9      
\"
    "inc r20                    
\"
    "brne 6f                    
\"
    "inc r21                    
\"
    "6:                         
\"
    "sts high_count+8, r20      
\"
    "sts high_count+9, r21      
\"
    "5:                         
\"
    :
    : [ch4_clear] "I" (~(1 << 4) & 0xFF), [ch4_set] "I" (1 << 4)
    : "r18","r19","r20","r21","r22","r23","r24","r25"
  );
#endif

#if NUM_CHANNELS >= 6
  asm volatile (
    // --- Channel 5 ---
\
    "lds r24, lfsr_state+10     
\"
    "lds r25, lfsr_state+11     
\"
    "clr r18                    
\"
    "sbrc r24, 0                
\"
    "ldi r18, 1                 
\"
    "lsr r24                    
\"
    "ror r25                    
\"
    "lds r22, lfsr_mask_current+10
\"
    "lds r23, lfsr_mask_current+11
\"
    "tst r18                    
\"
    "breq 1f                    
\"
    "eor r24, r22               
\"
    "eor r25, r23               
\"
    "1:                         
\"
    "sts lfsr_state+10, r24     
\"
    "sts lfsr_state+11, r25     
\"
    "andi r24, 0x01             
\"
    "mov r18, r24               
\"
    "andi r19, %[ch5_clear]     
\"
    "sbrc r18, 0                
\"
    "rjmp 2f                    
\"
    "ori r19, %[ch5_set]        
\"
    "2:                         
\"
    "lds r20, tick_count+10     
\"
    "lds r21, tick_count+11     
\"
    "inc r20                    
\"
    "brne 3f                    
\"
    "inc r21                    
\"
    "3:                         
\"
    "sts tick_count+10, r20     
\"
    "sts tick_count+11, r21     
\"
    "sbrc r18, 0                
\"
    "rjmp 4f                    
\"
    "rjmp 5f                    
\"
    "4:                         
\"
    "lds r20, high_count+10     
\"
    "lds r21, high_count+11     
\"
    "inc r20                    
\"
    "brne 6f                    
\"
    "inc r21                    
\"
    "6:                         
\"
    "sts high_count+10, r20     
\"
    "sts high_count+11, r21     
\"
    "5:                         
\"
    :
    : [ch5_clear] "I" (~(1 << 5) & 0xFF), [ch5_set] "I" (1 << 5)
    : "r18","r19","r20","r21","r22","r23","r24","r25"
  );
#endif

  // --- write aggregated PORTB value and epilogue ---
  asm volatile (
    "out _SFR_IO_ADDR(PORTB), r19
\"  // write PORTB with new channel bits
\
    "pop r25                    
\"  // restore state high (trash in some paths but safe)
\
    "pop r24                    
\"  // restore state low
\
    "pop r23                    
\"  // restore mask high
\
    "pop r22                    
\"  // restore mask low
\
    "pop r21                    
\"  // restore tmp high
\
    "pop r20                    
\"  // restore tmp low
\
    "pop r19                    
\"  // restore PORTB temp
\
    "pop r18                    
\"  // restore flag temp
\
    "pop r0                     
\"  // pop saved SREG
\
    "out __SREG__, r0           
\"  // restore SREG
\
    "reti                      
\"
    : : : "r0"
  );
}

// -------------------- Timer setup --------------------
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

// -------------------- Helpers --------------------
void apply_pwm_level_channel(uint8_t ch, uint8_t level) {
  if (ch >= NUM_CHANNELS) return;
  // In production: read mask from LUT based on level and store into lfsr_mask_current[ch]
  // For demo, pick a sample mask scaled by level (not meaningful), ensure non-zero state
  (void)level;
  lfsr_mask_current[ch] = 0xB400; // placeholder Galois-compatible mask
  if (lfsr_state[ch] == 0) lfsr_state[ch] = (1 << (ch+1)) | 1; // avoid all-zero
}

// -------------------- Setup & loop --------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  // Configure PORTB pins as outputs for channels
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    uint8_t bit = channel_mask_bits[i];
    DDRB |= (1 << bit);
    PORTB &= ~(1 << bit);
  }

  // Initialize demo masks/states
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    apply_pwm_level_channel(i, 128);
  }

  setup_timer1_ctc(TICK_HZ);
  Serial.println("Multi-channel optimized LFSR PWM ready");
}

void loop() {
  static uint32_t next_change_ms = 0;
  static uint8_t level = 0;
  uint32_t now = millis();

  if (now >= next_change_ms) {
    level += 32;
    for (uint8_t i = 0; i < NUM_CHANNELS; ++i) apply_pwm_level_channel(i, level);
    next_change_ms = now + 2000;
    Serial.print("Set level = "); Serial.println(level);
  }

  // Print per-channel measurement when tick_count reaches MEAS_TICKS
  noInterrupts();
  bool any_ready = false;
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    if (tick_count[i] >= MEAS_TICKS) { any_ready = true; break; }
  }
  if (any_ready) {
    // copy and reset
    uint16_t t[NUM_CHANNELS];
    uint16_t h[NUM_CHANNELS];
    for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
      t[i] = tick_count[i];
      h[i] = high_count[i];
      tick_count[i] = 0;
      high_count[i] = 0;
    }
    interrupts();

    for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
      float achieved = (t[i] == 0) ? 0.0f : ((float)h[i] / (float)t[i]);
      Serial.print("CH"); Serial.print(i);
      Serial.print(" -> "); Serial.print(achieved * 100.0f, 2);
      Serial.print("% (ticks="); Serial.print(t[i]); Serial.print(", high="); Serial.print(h[i]); Serial.println(")");
    }
  } else {
    interrupts();
  }

  // serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+') {
      // quick bump all channels
      for (uint8_t i = 0; i < NUM_CHANNELS; ++i) apply_pwm_level_channel(i, min(255, 128+16));
    }
  }
  delay(10);
}

/*
  Notes:
  - This implementation favors minimal ISR cost over flexibility. Keeping all
    channels on PORTB allows a single atomic write to update outputs.
  - To extend to arbitrary pins spanning multiple ports you have two options:
      1) extend the unrolled ISR to compute and write each involved PORTx (more complex)
      2) keep the LFSR step in assembly but perform per-channel port writes in C (higher overhead)
  - If you want, I can now:
      - convert tick/high counters to 32-bit and provide a micro-optimized version
      - extend support to use arbitrary pins across PORTB/PORTD with minimal added cycles
      - plug your precomputed 256-entry LUT (PROGMEM) and implement per-channel level selection
*/

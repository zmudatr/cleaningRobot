#include "IRremote.h"

//+=============================================================================
// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
//
bool IRrecv::decode() {
    if (irparams.rcvstate != IR_REC_STATE_STOP) {
        return false;
    }

    /*
     * First copy 3 values from irparams to internal results structure
     */
    results.rawbuf = irparams.rawbuf;
    results.rawlen = irparams.rawlen;
    results.overflow = irparams.overflow;

#if DECODE_NEC_STANDARD
    DBG_PRINTLN("Attempting NEC_STANDARD decode");
    if (decodeNECStandard()) {
        return true;
    }
#endif

#if DECODE_NEC
    DBG_PRINTLN("Attempting NEC decode");
    if (decodeNEC()) {
        return true;
    }
#endif


    // Throw away and start over
    resume();
    return false;
}

/*
 * For backwards compatibility
 */
bool IRrecv::decode(decode_results *aResults) {
    if (irparams.rcvstate != IR_REC_STATE_STOP) {
        return false;
    }

    /*
     * First copy 3 values from irparams to internal results structure
     */
    results.rawbuf = irparams.rawbuf;
    results.rawlen = irparams.rawlen;
    results.overflow = irparams.overflow;

#if DECODE_NEC
    DBG_PRINTLN("Attempting NEC decode");
    if (decodeNEC(aResults)) {
        return true;
    }
#endif

#if defined(DECODE_HASH)
    DBG_PRINTLN("Hash decode");
    // decodeHash returns a hash on any input.
    // Thus, it needs to be last in the list.
    // If you add any decodes, add them before this.
    if (decodeHash(aResults)) {
        return true;
    }
#endif

    // Throw away and start over
    resume();
    return false;
}

//+=============================================================================
IRrecv::IRrecv(int recvpin) {
    irparams.recvpin = recvpin;
    irparams.blinkflag = 0;
}

IRrecv::IRrecv(int recvpin, int blinkpin) {
    irparams.recvpin = recvpin;
    irparams.blinkpin = blinkpin;
    pinMode(blinkpin, OUTPUT);
    irparams.blinkflag = 0;
}

//+=============================================================================
// initialization
//
#ifdef USE_DEFAULT_ENABLE_IR_IN
void IRrecv::enableIRIn() {
// the interrupt Service Routine fires every 50 uS
    noInterrupts();
    // Setup pulse clock timer interrupt
    // Prescale /8 (16M/8 = 0.5 microseconds per tick)
    // Therefore, the timer interval can range from 0.5 to 128 microseconds
    // Depending on the reset value (255 to 0)
    timerConfigForReceive();

    // Timer2 Overflow Interrupt Enable
    TIMER_ENABLE_RECEIVE_INTR;

    TIMER_RESET_INTR_PENDING;

    interrupts();

    // Initialize state machine state
    irparams.rcvstate = IR_REC_STATE_IDLE;
    //    irparams.rawlen = 0; // not required

    // Set pin modes
    pinMode(irparams.recvpin, INPUT);
}

void IRrecv::disableIRIn() {
    TIMER_DISABLE_RECEIVE_INTR;
}

#endif // USE_DEFAULT_ENABLE_IR_IN

//+=============================================================================
// Enable/disable blinking of pin 13 on IR processing
//
void IRrecv::blink13(int blinkflag) {
#ifdef BLINKLED
    irparams.blinkflag = blinkflag;
    if (blinkflag) {
        pinMode(BLINKLED, OUTPUT);
    }
#endif
}

//+=============================================================================
// Return if receiving new IR signals
//
bool IRrecv::isIdle() {
    return (irparams.rcvstate == IR_REC_STATE_IDLE || irparams.rcvstate == IR_REC_STATE_STOP) ? true : false;
}

bool IRrecv::available() {
    if (irparams.rcvstate != IR_REC_STATE_STOP) {
        return false;
    }
    results.rawbuf = irparams.rawbuf;
    results.rawlen = irparams.rawlen;

    results.overflow = irparams.overflow;
    if (!results.overflow) {
        return true;
    }
    resume(); //skip overflowed buffer
    return false;
}

//+=============================================================================
// Restart the ISR state machine
//
void IRrecv::resume() {
    irparams.rcvstate = IR_REC_STATE_IDLE;
//    irparams.rawlen = 0; // not required
}

# if DECODE_HASH
//+=============================================================================
// hashdecode - decode an arbitrary IR code.
// Instead of decoding using a standard encoding scheme
// (e.g. Sony, NEC, RC5), the code is hashed to a 32-bit value.
//
// The algorithm: look at the sequence of MARK signals, and see if each one
// is shorter (0), the same length (1), or longer (2) than the previous.
// Do the same with the SPACE signals.  Hash the resulting sequence of 0's,
// 1's, and 2's to a 32-bit value.  This will give a unique value for each
// different code (probably), for most code systems.
//
// http://arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
//
// Compare two tick values, returning 0 if newval is shorter,
// 1 if newval is equal, and 2 if newval is longer
// Use a tolerance of 20%
//
int IRrecv::compare(unsigned int oldval, unsigned int newval) {
    if (newval * 10 < oldval * 8) {
        return 0;
    }
    if (oldval * 10 < newval * 8) {
        return 2;
    }
    return 1;

}

/*
 * Each bit looks like: MARK + SPACE_1 -> 1
 *                 or : MARK + SPACE_0 -> 0
 * Data is read MSB first.
 */
unsigned long IRrecv::decodePulseDistanceData(uint8_t aNumberOfBits, uint8_t aStartOffset, unsigned int aBitMarkMicros,
        unsigned int aOneSpaceMicros, unsigned int aZeroSpaceMicros) {
    unsigned long aDecodedData = 0;

    for (uint8_t i = 0; i < aNumberOfBits; i++) {
        // Check for constant length mark
        if (!MATCH_MARK(results.rawbuf[aStartOffset], aBitMarkMicros)) {
            return false;
        }
        aStartOffset++;

        // Check for variable length space indicating a 0 or 1
        if (MATCH_SPACE(results.rawbuf[aStartOffset], aOneSpaceMicros)) {
            aDecodedData = (aDecodedData << 1) | 1;
        } else if (MATCH_SPACE(results.rawbuf[aStartOffset], aZeroSpaceMicros)) {
            aDecodedData = (aDecodedData << 1) | 0;
        } else {
            return false;
        }
        aStartOffset++;
    }
    return aDecodedData;
}

//+=============================================================================
// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
// Converts the raw code values into a 32-bit hash code.
// Hopefully this code is unique for each button.
// This isn't a "real" decoding, just an arbitrary value.
//
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

bool IRrecv::decodeHash() {
    long hash = FNV_BASIS_32;

    // Require at least 6 samples to prevent triggering on noise
    if (results.rawlen < 6) {
        return false;
    }

    for (unsigned int i = 1; (i + 2) < results.rawlen; i++) {
        int value = compare(results.rawbuf[i], results.rawbuf[i + 2]);
        // Add value into the hash
        hash = (hash * FNV_PRIME_32) ^ value;
    }

    results.value = hash;
    results.bits = 32;
    results.decode_type = UNKNOWN;

    return true;
}
bool IRrecv::decodeHash(decode_results *aResults) {
    bool aReturnValue = decodeHash();
    *aResults = results;
    return aReturnValue;
}
#endif // defined(DECODE_HASH)

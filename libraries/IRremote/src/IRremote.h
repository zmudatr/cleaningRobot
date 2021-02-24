/**
 * @file IRremote.h
 * @brief Public API to the library.
 */

//******************************************************************************
// IRremote
// Version 2.0.1 June, 2015
// Copyright 2009 Ken Shirriff
// For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
// Edited by Mitra to add new controller SANYO
//
// Interrupt code based on NECIRrcv by Joe Knapp
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
// Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
//
// JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
// LG added by Darryl Smith (based on the JVC protocol)
// Whynter A/C ARC-110WD added by Francesco Meschia
// MagiQuest added by E. Stuart Hicks (based on code by mpflaga - https://github.com/mpflaga/Arduino-IRremote/)
//******************************************************************************
#ifndef IRremote_h
#define IRremote_h

//------------------------------------------------------------------------------
// The ISR header contains several useful macros the user may wish to use
//
#include "private/IRremoteInt.h"

#ifdef ARDUINO_ARCH_AVR
#include <avr/pgmspace.h>
#define HAS_FLASH_READ 1
#define STRCPY_PF_CAST(x) (x)
#else
#define HAS_FLASH_READ 0
#endif

/****************************************************
 *                     PROTOCOLS
 ****************************************************/


//#define USE_NEC_STANDARD // remove comment to have the standard NEC decoding (LSB first) available.
#if defined(USE_NEC_STANDARD)
#define DECODE_NEC_STANDARD  1
#define DECODE_NEC           0
#define LSB_FIRST_REQURED
#else
#define DECODE_NEC_STANDARD  0
#define DECODE_NEC           1
#endif
#define SEND_NEC             1
#define SEND_NEC_STANDARD    1


#define DECODE_HASH          1 // special decoder for all protocols

/**
 * An enum consisting of all supported formats.
 * You do NOT need to remove entries from this list when disabling protocols!
 */
typedef enum {
    UNKNOWN = -1,
    UNUSED = 0,
    NEC_STANDARD,
    NEC,
    SONY,
    PANASONIC,
    JVC,
    SAMSUNG,
    WHYNTER,
    AIWA_RC_T501,
    LG,
    SANYO,
    MITSUBISHI,
    DISH,
    SHARP,
    SHARP_ALT,
    DENON,
    LEGO_PF,
    BOSEWAVE,
    MAGIQUEST,
} decode_type_t;

/**
 * Set DEBUG to 1 for lots of lovely debug output.
 */
#define DEBUG  0

//------------------------------------------------------------------------------
// Debug directives
//
#if DEBUG
#  define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
#  define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
/**
 * If DEBUG, print the arguments, otherwise do nothing.
 */
#  define DBG_PRINT(...) void()
/**
 * If DEBUG, print the arguments as a line, otherwise do nothing.
 */
#  define DBG_PRINTLN(...) void()
#endif

//------------------------------------------------------------------------------
// Helper macro for getting a macro definition as string
//
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

//------------------------------------------------------------------------------
// Mark & Space matching functions
//
int MATCH(int measured, int desired);
int MATCH_MARK(int measured_ticks, int desired_us);
int MATCH_SPACE(int measured_ticks, int desired_us);

/****************************************************
 *                     RECEIVING
 ****************************************************/
/**
 * Results returned from the decoder
 */
struct decode_results {
    decode_type_t decode_type;  ///< UNKNOWN, NEC, SONY, RC5, ...
    unsigned int address;       ///< Used by Panasonic & Sharp [16-bits]
    unsigned long value;        ///< Decoded value [max 32-bits]
    int bits;                   ///< Number of bits in decoded value
    unsigned int magnitude;     ///< Used by MagiQuest [16-bits]

    // next 3 values are copies of irparams values
    unsigned int *rawbuf;       ///< Raw intervals in 50uS ticks
    unsigned int rawlen;        ///< Number of records in rawbuf
    bool overflow;               ///< true if IR raw code too long
};

/**
 * Decoded value for NEC when a repeat code is received
 */
#define REPEAT 0xFFFFFFFF

/**
 * Main class for receiving IR
 */
class IRrecv {
public:
    /**
     * Instantiate the IRrecv class. Multiple instantiation is not supported.
     * @param recvpin Arduino pin to use. No sanity check is made.
     */
    IRrecv(int recvpin);
    /**
     * Instantiate the IRrecv class. Multiple instantiation is not supported.
     * @param recvpin Arduino pin to use, where a demodulating IR receiver is connected.
     * @param blinkpin pin to blink when receiving IR. Not supported by all hardware. No sanity check is made.
     */
    IRrecv(int recvpin, int blinkpin);

    /**
     * TODO: Why is this public???
     * @param blinkflag
     */
    void blink13(int blinkflag);

    /**
     * Attempt to decode the recently receive IR signal
     * @param results decode_results instance returning the decode, if any.
     * @return success of operation.
     */
    bool decode(decode_results *aResults);
    bool decode();

    /**
     * Enable IR reception.
     */
    void enableIRIn();

    /**
     * Disable IR reception.
     */
    void disableIRIn();

    /**
     * Returns status of reception
     * @return true if no reception is on-going.
     */
    bool isIdle();

    /**
     * Returns status of reception and copies IR-data to decode_results buffer if true.
     * @return true if data is available.
     */
    bool available();

    /**
     * Called to re-enable IR reception.
     */
    void resume();

    /**
     * Print the result (second argument) as Pronto Hex on the Stream supplied as argument.
     * @param stream The Stream on which to write, often Serial
     * @param results the decode_results as delivered from irrecv.decode.
     * @param frequency Modulation frequency in Hz. Often 38000Hz.
     */
    void dumpPronto(Stream& stream, unsigned int frequency = 38000U);

    unsigned long decodePulseDistanceData(uint8_t aNumberOfBits, uint8_t aStartOffset,
            unsigned int aBitMarkMicros, unsigned int aOneSpaceMicros, unsigned int aZeroSpaceMicros);

    decode_results results; // the instance for decoding

private:
#if DECODE_HASH
    bool decodeHash();
    bool decodeHash(decode_results *aResults);
    int compare(unsigned int oldval, unsigned int newval);
#endif

    //......................................................................
#if DECODE_NEC
    bool decodeNEC();
    bool decodeNEC(decode_results *aResults);
#endif
#if DECODE_NEC_STANDARD
    bool decodeNECStandard();
#endif

};

/****************************************************
 *                     SENDING
 ****************************************************/
/**
 * Define to use no carrier PWM, just simulate a receiver signal.
 */
//#define USE_NO_SEND_PWM

/**
 * Define to use carrier PWM generation in software, instead of hardware PWM.
 */
//#define USE_SOFT_SEND_PWM

/**
 * Define to use spin wait instead of delayMicros() for USE_SOFT_SEND_PWM.
 */
//#define USE_SPIN_WAIT

/**
 * Main class for sending IR
 */
class IRsend {
public:
#if defined(USE_SOFT_SEND_PWM) || defined(USE_NO_SEND_PWM)
    IRsend(int pin = IR_SEND_PIN) {
      sendPin = pin;
    }
#else
    IRsend() {
    }
#endif

    void custom_delay_usec(unsigned long uSecs);
    void enableIROut(int khz);
    void sendPulseDistanceWidthData(unsigned int aOneMarkMicros, unsigned int aOneSpaceMicros, unsigned int aZeroMarkMicros,
            unsigned int aZeroSpaceMicros, unsigned long aData, uint8_t aNumberOfBits, bool aMSBfirst = true);
    void mark(unsigned int usec);
    void space(unsigned int usec);
    void sendRaw(const unsigned int buf[], unsigned int len, unsigned int hz);
    void sendRaw_P(const unsigned int buf[], unsigned int len, unsigned int hz);

    //......................................................................
#if SEND_NEC
    void sendNEC(unsigned long data, int nbits, bool repeat = false);
#endif
#if SEND_NEC_STANDARD
    void sendNECStandard(unsigned long data, int nbits, bool repeat = false);
#endif
    //......................................................................

    /**
     * Parse the string given as Pronto Hex, and send it a number of times given
     * as the second argument. Thereby the division of the Pronto Hex into
     * an intro-sequence and a repeat sequence is taken into account:
     * First the intro sequence is sent, then the repeat sequence is sent times-1 times.
     * However, if the intro sequence is empty, the repeat sequence is sent times times.
     * <a href="http://www.harctoolbox.org/Glossary.html#ProntoSemantics">Reference</a>.
     *
     * Note: Using this function is very wasteful for the memory consumption on
     * a small board.
     * Normally it is a much better ide to use a tool like e.g. IrScrutinizer
     * to transform Pronto type signals offline
     * to a more memory efficient format.
     *
     * @param prontoHexString C type string (null terminated) containing a Pronto Hex representation.
     * @param times Number of times to send the signal.
     */
    void sendPronto(const char* prontoHexString, unsigned int times = 1U);

    void sendPronto(const uint16_t* data, unsigned int length, unsigned int times = 1U);

#if HAS_FLASH_READ || defined(DOXYGEN)
    void sendPronto_PF(uint_farptr_t str, unsigned int times = 1U);

    /**
     * Version of sendPronto that reads from PROGMEM, saving RAM memory.
     * @param pronto C type string (null terminated) containing a Pronto Hex representation.
     * @param times Number of times to send the signal.
     */
    void sendPronto_PF(const char *str, unsigned int times = 1U);
    void sendPronto(const __FlashStringHelper *str, unsigned int times = 1U);
#endif

private:

#if defined(USE_SOFT_SEND_PWM) || defined(USE_NO_SEND_PWM)
    int sendPin;

#  if defined(USE_SOFT_SEND_PWM)
    unsigned int periodTime;
    unsigned int periodOnTime;

    void sleepMicros(unsigned long us);
    void sleepUntilMicros(unsigned long targetTime);
#  endif

#else
    const int sendPin = IR_SEND_PIN;
#endif
};

#endif // IRremote_h

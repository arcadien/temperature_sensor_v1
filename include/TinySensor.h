#pragma once

#include <qpn.h>     // QP-nano framework
#include <Arduino.h> // Arduino API

#define OREGON_MODE 0
#include <oregon.h> // 433Mhz transmission library
#include <x10rf.h>
#include <avr/wdt.h>
#include <ISR.h>

// define to println() many information
#define TRACE 1

// ------------

// number of system clock ticks in one second
#define BSP_TICKS_PER_SEC 100

// the pin number of the on-board LED
#define LED_PIN 3

// the pin number which activates the vcc rail for peripherals (emitter, sensors)
#define PERIPH_VCC_PIN 10

// the pin for 433Mhz data
#define TX_PIN 5

// some constant for temperature sensor
#define ANALOG_TEMP_SENSOR_PIN A0

// 2^4, allow division with bitshift
// ratio to apply to the raw analog measurement from ADC on LM38
const uint8_t TEMP_SAMPLE_COUNT = 16;

// below this voltage (mV), we
// consider beiing in low battery level.
const long LOW_BATTERY = 1100 * 3;

// see https://playground.arduino.cc/Main/LM35HigherResolution
#define AREF 1.103800587
#define TEMPERATURE_RATIO (AREF / 1024.0 * 100.0)

// emit each 5 minutes (how many 8sec watchdog wakeups)
const uint8_t SLEEP_DELAY_SECOND = (uint8_t)((5 * 60) / 8);

uint8_t OREGON_TYPE[] = {0xEA, 0x4C}; // inside temp
#define OREGON_ID 0xBC
#define COMMAND_REPEAT_COUNT 5

#define rfxSensorID 13

class TinySensor
{
public:
    inline void Setup()
    {
#if defined(__AVR_ATmega32U4__)
        MCUCR |= (1 << JTD);
#endif
        Serial.begin(57600);

        _oregon.setType(_oregonMessage, OREGON_TYPE);
        _oregon.setChannel(_oregonMessage, Oregon::Channel::ONE);
        _oregon.setId(_oregonMessage, OREGON_ID);

        pinMode(13, OUTPUT);
        pinMode(LED_PIN, OUTPUT);
        pinMode(PERIPH_VCC_PIN, OUTPUT);
        pinMode(TX_PIN, OUTPUT);
        pinMode(ANALOG_TEMP_SENSOR_PIN, INPUT);

        digitalWrite(13, LOW);

        // the state machine will
        // drop in SENSING state
        // after boot
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(PERIPH_VCC_PIN, HIGH);
        PrintInfo();
    }

    inline void BeforeIdle()
    {
        // clear various "reset" flags
        MCUSR = 0;

        cli();
        WDTCSR |= (1 << WDCE) | (1 << WDE);
        // 8 seconds duration, interrupt
        WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
        wdt_reset();
#if defined(__AVR_ATmega328P__)
        // turn off brown‐out enable in software
        MCUCR = (1 << BODS) | (1 << BODSE);
        MCUCR = (1 << BODS);
#endif
        sei();
    }

    inline void ReadTemperature()
    {
        // Battery level and LM35
        // will use internal 1v1 reference
        analogReference(INTERNAL);

        // first read is usually false
        // make a blank read
        analogRead(ANALOG_TEMP_SENSOR_PIN);

        uint16_t accumulator = 0;
        for (uint8_t j = 0; j < TEMP_SAMPLE_COUNT; ++j)
        {
            accumulator += analogRead(ANALOG_TEMP_SENSOR_PIN);
        }

        _temperature = (accumulator >> 4);

        // the value is adjusted with Aref value for each sensor module.
        _temperature *= TEMPERATURE_RATIO;

#ifdef TRACE
        Serial.print(F("Temp: "));
        Serial.print(_temperature);
        Serial.println(F("°c"));
#endif

        _oregon.setTemperature(_oregonMessage, _temperature);
    }

    inline void SendBatteryLevel()
    {
        SuspendHeartbeat();
        _x10.RFXmeter(rfxSensorID, 0, _rawBattery);
        ResumeHeartbeat();
#ifdef TRACE
        Serial.println(F("Battery level sent"));
#endif
    }

    inline void SuspendHeartbeat()
    {
#if defined(__AVR_ATmega328P__)
        TIMSK2 &= ~(1U << OCIE2A); // cut heartbeat to avoid IT during transmission

#elif defined(__AVR_ATmega32U4__)
        TIMSK3 &= ~(1U << OCIE3A);
#endif
    }
    inline void ResumeHeartbeat()
    {
#if defined(__AVR_ATmega328P__)
        TIMSK2 |= (1U << OCIE2A); // re-enable heartbeat

#elif defined(__AVR_ATmega32U4__)
        TIMSK3 |= (1U << OCIE3A);
#endif
    }

    inline void SendTemperature()
    {
        SuspendHeartbeat();

        _oregon.calculateAndSetChecksum(_oregonMessage);

        _oregon.txPinLow();
        _delay_us(_oregon.TWOTIME * 8);

        _oregon.sendOregon(_oregonMessage, sizeof(_oregonMessage));

        // pause before new transmission
        _oregon.txPinLow();
        _delay_us(_oregon.TWOTIME * 8);

        _oregon.sendOregon(_oregonMessage, sizeof(_oregonMessage));

        ResumeHeartbeat();

#ifdef TRACE
        Serial.println(F("Temperature sent"));
#endif
    }

    inline void ReadVcc()
    {

        byte ADMUX_backup = ADMUX;
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
        _delay_ms(2);        // Wait for Vref to settle
        ADCSRA |= _BV(ADSC); // Start conversion
        while (bit_is_set(ADCSRA, ADSC))
            ; // measuring

        // scale_constant = internal1.1Ref * 1023 * 1000
        // internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)

        long result = 1126400L / ADC; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000

        ADMUX = ADMUX_backup;
        _delay_ms(2);

#ifdef TRACE
        Serial.print(F("VCC: "));
        Serial.print(result);
        Serial.println(F("mV"));
#endif

        _rawBattery = result;
        _oregon.setBatteryLevel(_oregonMessage, (result < LOW_BATTERY) ? 0 : 1);
    }

    inline void ToString()
    {
        Serial.println(F("Temp sensor data: "));
        Serial.print(F("Temperature: "));
        Serial.println(_temperature);
        Serial.print(F("Battery: "));
        Serial.print(_rawBattery);
        Serial.println(F("mV"));
    }

    inline void PowerUp()
    {
#if defined(__AVR_ATmega32U4__)
        PRR0 &= ~(1U << PRTWI);
        PRR0 &= ~(1U << PRTIM0);
        PRR0 &= ~(1U << PRTIM1);
        PRR0 &= ~(1U << PRSPI);
        PRR0 &= ~(1U << PRADC);

        // PRR1 &= ~(1U << PRUSB);
        PRR1 &= ~(1U << PRTIM3);
        PRR1 &= ~(1 << PRUSART1);
        ADCSRA |= (1U << ADEN);

#elif defined(__AVR_ATmega328P__)
        PRR &= (1U << PRTWI);
        PRR &= (1U << PRTIM2);
        PRR &= (1U << PRTIM0);
        PRR &= (1U << PRTIM1);
        PRR &= (1U << PRSPI);
        PRR &= (1U << PRUSART0);
        ADCSRA |= (1U << ADEN);
        PRR &= ~(1U << PRADC);

#endif
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(PERIPH_VCC_PIN, HIGH);

        Serial.begin(57600);
        _x10.begin();

        // http://www.ti.com/lit/ds/symlink/lm35.pdf p.12
        // (startup response)
        _delay_us(30);
    }

    inline void PowerDown()
    {
        Serial.flush();
        _delay_ms(5);
        digitalWrite(LED_PIN, LOW);
        digitalWrite(PERIPH_VCC_PIN, LOW);

#if defined(__AVR_ATmega32U4__)
        PRR0 |= (1 << PRTWI);
        PRR0 |= (1 << PRTIM0);
        PRR0 |= (1 << PRTIM1);
        PRR0 |= (1 << PRSPI);
        ADCSRA &= ~(1 << ADEN);
        PRR0 |= (1 << PRADC);

        // PRR1 |= (1 << PRUSB);
        PRR1 |= (1 << PRTIM3);
        PRR1 |= (1 << PRUSART1);

#elif defined(__AVR_ATmega328P__)
        PRR |= (1U << PRTWI);
        PRR |= (1U << PRTIM2);
        PRR |= (1U << PRTIM0);
        PRR |= (1U << PRTIM1);
        PRR |= (1U << PRSPI);
        PRR |= (1U << PRUSART0);
        ADCSRA &= ~(1U << ADEN);
        PRR |= (1U << PRADC);

#endif
    }

    /*!
    * Setup system so that an interrupt will
    * occur at a frenquency of 100hz. That 
    * interrupt will be used as QP tick source.
    */
    inline void SetupHeatbeat()
    {
#if defined(__AVR_ATmega32U4__)
        // set Timer3 in CTC mode, 1/1024 prescaler, start the timer ticking...
        TCCR3A |= (1U << COM3A1) | (1U << COM3A0) | (1U << WGM32);
        TCCR3B |= (1U << WGM32) | (1U << CS32) | (1U << CS30); //1/1024
        TCNT3 = 0;
        OCR3AL = (F_CPU / BSP_TICKS_PER_SEC / 1024U) - 1U;
        TIMSK3 |= (1U << OCIE3A);

#elif defined(__AVR_ATmega328P__)

        // set Timer2 in CTC mode, 1/1024 prescaler, start the timer ticking...
        TCCR2A = (1U << WGM21) | (0U << WGM20);
        TCCR2B = (1U << CS22) | (1U << CS21) | (1U << CS20); // 1/2^10
        ASSR &= ~(1U << AS2);
        TIMSK2 = (1U << OCIE2A); // enable TIMER2 compare Interrupt
        TCNT2 = 0U;

        // set the output-compare register based on the desired tick frequency
        OCR2A = (F_CPU / BSP_TICKS_PER_SEC / 1024U) - 1U;

#endif
    }

    inline void PrintInfo()
    {
        Serial.print(F("QP-nano: "));
        Serial.println(F(QP_VERSION_STR));

        Serial.print(F("CPU Freq: "));
        Serial.print(F_CPU / 1000000);
        Serial.println("Mhz");

        Serial.print(F("Watchdog overflow count before emission: "));
        Serial.println(SLEEP_DELAY_SECOND);

        Serial.print(F("State machine ticks/s: "));
        Serial.println(BSP_TICKS_PER_SEC);
    }

private:
    float _temperature;
    long _rawBattery;
    Oregon _oregon;
    x10rf _x10 = x10rf(TX_PIN, LED_PIN, COMMAND_REPEAT_COUNT);

    // Buffer for Oregon message
#if OREGON_MODE == MODE_0
    uint8_t _oregonMessage[8] = {};
#elif OREGON_MODE == MODE_1
    uint8_t _oregonMessage[9] = {};
#elif OREGON_MODE == MODE_2
    uint8_t _oregonMessage[11] = {};
#else
#error mode unknown
#endif
};
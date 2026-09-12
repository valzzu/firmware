#include "variant.h"

// Display CS and touch CS share the SPI bus with the SD card. Deselect both before the SD
// card is probed, otherwise the touch controller can leave MISO driven and the card is never
// detected.
void earlyInitVariant()
{
    pinMode(LGFX_PIN_CS, OUTPUT);
    digitalWrite(LGFX_PIN_CS, HIGH);
    pinMode(LGFX_TOUCH_CS, OUTPUT);
    digitalWrite(LGFX_TOUCH_CS, HIGH);
    pinMode(SDCARD_CS, OUTPUT);
    digitalWrite(SDCARD_CS, HIGH);
}

#ifndef LED_INDICATION_H
#define LED_INDICATION_H

#include <Adafruit_NeoPixel.h>

class SingleNeoPixel {
private:
    Adafruit_NeoPixel strip;
    int ledIndex = 0;
    uint32_t lastColor;
    uint8_t brightness;

    static constexpr uint32_t PREDEFINED_COLORS[] = {
        0xFF0000, // RED
        0x00FF00, // GREEN
        0x0000FF, // BLUE
        0xFFFF00, // YELLOW
        0xFFFFFF, // WHITE
        0x000000  // OFF
    };

public:
    enum Color {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        WHITE,
        OFF
    };

    SingleNeoPixel(int pin, uint8_t brightness=10) : lastColor(0x000000){
        strip = Adafruit_NeoPixel(1, pin, NEO_GRB + NEO_KHZ800);
        this->brightness = brightness;
    }

    void begin() {
        strip.begin();
        strip.setBrightness(brightness);
        strip.show();
    }

    void setColor(uint8_t red, uint8_t green, uint8_t blue) {
        uint32_t color = strip.Color(red, green, blue);
        if (color != lastColor) {
            strip.setPixelColor(ledIndex, color);
            strip.show();
            lastColor = color;
        }
    }

    void setColor(Color color) {
        uint32_t colorValue = PREDEFINED_COLORS[color];
        if (colorValue != lastColor) {
            strip.setPixelColor(ledIndex, colorValue);
            strip.show();
            lastColor = colorValue;
        }
    }

    void off() {
        setColor(OFF);
    }
};


#endif // LED_INDICATION_H

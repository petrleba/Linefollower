#include <Arduino.h>

// --- KONFIGURACE ---
// Pin, na kterém je připojena RGB LED. Pro vaši desku je to GPIO 48.
#define ONBOARD_RGB_PIN 48

void setup()
{
  Serial.begin(115200);

  // Dlouhá pauza, abyste stihli otevřít sériový monitor po nahrání.
  delay(5000);
  Serial.println("--- START TESTU RGB LED ---");
  Serial.println("Pokud vidíte tento text, sériový port funguje správně.");
  Serial.printf("Pokouším se ovládat LED na pinu: %d\n", ONBOARD_RGB_PIN);
}

void loop()
{
  Serial.println("Barva: Červená");
  // Volání funkce s parametry: pin, R, G, B (jas 0-255).
  neopixelWrite(ONBOARD_RGB_PIN, 64, 0, 0);
  delay(2000);

  Serial.println("Barva: Zelená");
  neopixelWrite(ONBOARD_RGB_PIN, 0, 64, 0);
  delay(2000);

  Serial.println("Barva: Modrá");
  neopixelWrite(ONBOARD_RGB_PIN, 0, 0, 64);
  delay(2000);

  Serial.println("Barva: Vypnuto");
  neopixelWrite(ONBOARD_RGB_PIN, 0, 0, 0);
  delay(2000);
}

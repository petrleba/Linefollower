#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "esp_random.h"

char chyba[20];
char uhel[20];
const int OLED_WIDTH = 128;

// --- Definice pinů ---
#define OLED_SCLK 40           // SCL pin
#define OLED_MOSI 41           // SDA pin
#define OLED_DC 39             // DC pin
#define OLED_RST U8X8_PIN_NONE // Reset je připojen na napájení
#define OLED_CS U8X8_PIN_NONE  // CS pin není přítomen

// --- Vytvoření instance U8g2 ---
// Toto je vaše "nejlepší sázka" pro začátek.
// Ovladač: SSD1306, Rozlišení: 128X64, Varianta: NONAME, Buffer: F (Full), Rozhraní: 4-vodičové Hardwarové SPI
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RST);

void setup()
{
  Serial.println("Standardní SPI");
  Serial.begin(115200);

  // Inicializace SPI sběrnice na vlastních pinech.
  // Toto je nutné udělat PŘED u8g2.begin() pro hardwarové SPI na ESP32-S3.
  SPI.begin(OLED_SCLK, -1, OLED_MOSI, -1);

  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("CS: ");
  Serial.println(SS);

  u8g2.begin();
}

void loop()
{
  auto sensor01 = random(0, 1024);
  auto sensor02 = random(0, 1024);
  auto sensor03 = random(0, 1024);
  auto sensor04 = random(0, 1024);
  auto sensor05 = random(0, 1024);
  auto sensor06 = random(0, 1024);
  auto sensor07 = random(0, 1024);
  auto sensor08 = random(0, 1024);
  auto sensor09 = random(0, 1024);
  auto sensor10 = random(0, 1024);
  auto sensor11 = random(0, 1024);
  auto sensor12 = random(0, 1024);
  auto sensor13 = random(0, 1024);
  auto sensor14 = random(0, 1024);
  auto sensor15 = random(0, 1024);
  auto sensor16 = random(0, 1024);
  int error = round((sensor01 + sensor02 + sensor03 + sensor04 + sensor05 + sensor06 + sensor07 + sensor08 - sensor09 - sensor10 - sensor11 - sensor12 - sensor13 - sensor14 - sensor15 - sensor16) / 16);
  auto angle = random(0, 360);

  u8g2.clearBuffer();
  u8g2.setDrawColor(1);

  u8g2.drawBox(1, 64 - round(sensor01 / 22), 7, round(sensor01 / 22));
  u8g2.drawBox(9, 64 - round(sensor02 / 22), 7, round(sensor02 / 22));
  u8g2.drawBox(17, 64 - round(sensor03 / 22), 7, round(sensor03 / 22));
  u8g2.drawBox(25, 64 - round(sensor04 / 22), 7, round(sensor04 / 22));
  u8g2.drawBox(33, 64 - round(sensor05 / 22), 7, round(sensor05 / 22));
  u8g2.drawBox(41, 64 - round(sensor06 / 22), 7, round(sensor06 / 22));
  u8g2.drawBox(49, 64 - round(sensor07 / 22), 7, round(sensor07 / 22));
  u8g2.drawBox(57, 64 - round(sensor08 / 22), 7, round(sensor08 / 22));
  u8g2.drawBox(65, 64 - round(sensor09 / 22), 7, round(sensor09 / 22));
  u8g2.drawBox(73, 64 - round(sensor10 / 22), 7, round(sensor10 / 22));
  u8g2.drawBox(81, 64 - round(sensor11 / 22), 7, round(sensor11 / 22));
  u8g2.drawBox(89, 64 - round(sensor12 / 22), 7, round(sensor12 / 22));
  u8g2.drawBox(97, 64 - round(sensor13 / 22), 7, round(sensor13 / 22));
  u8g2.drawBox(105, 64 - round(sensor14 / 22), 7, round(sensor14 / 22));
  u8g2.drawBox(113, 64 - round(sensor15 / 22), 7, round(sensor15 / 22));
  u8g2.drawBox(121, 64 - round(sensor16 / 22), 7, round(sensor16 / 22));

  u8g2.setFont(u8g2_font_inb16_mf);
  sprintf(chyba, "%d", error);
  int x = OLED_WIDTH - u8g2.getStrWidth(chyba);
  u8g2.drawStr(x, 16, chyba);

  sprintf(uhel, "%d ", angle);
  x = round(OLED_WIDTH / 2) - u8g2.getStrWidth(uhel) + 1;
  u8g2.drawStr(x, 16, uhel);

  u8g2.sendBuffer();

  delay(100);
  // dát tam chybu a pak úhel natočení
}

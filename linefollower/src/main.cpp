#include <Arduino.h>
#include "CytronMotorDriver.h"

// Zapínání a vypínání debugování: výpisů, reakce motorů
bool motorsOn = true;
bool debugOn = true;
bool debugCalibration = true;
bool debugAnalysis = true;
bool debugResultsAnalysis = true;

// Konfigurace motorů
CytronMD motorL(PWM_PWM, 19, 18);
CytronMD motorR(PWM_PWM, 16, 17);

// Piny senzorů
const int NUM_SENSORS = 12;
const int sensorPins[] = {27, 35, 26, 32, 25, 33, 34, 14, 39, 12, 36, 13};
/*
LEVÁ STRANA
sensor  9 = GPIO 27 = index  0
sensor 10 = GPIO 35 = index  1
sensor 11 = GPIO 26 = index  2
sensor 12 = GPIO 32 = index  3
sensor 13 = GPIO 25 = index  4
sensor 14 = GPIO 33 = index  5
PRAVÁ STRANA
sensor  3 = GPIO 13 = index 11
sensor  4 = GPIO 36 = index 10
sensor  5 = GPIO 12 = index  9
sensor  6 = GPIO 39 = index  8
sensor  7 = GPIO 14 = index  7
sensor  8 = GPIO 34 = index  6
*/

// Váhy pro senzory od -5500 do +5500 - rozestup vah je stejný pro všechny, protože jejich vzájemná vzdálenost je také stejná (8mm)
const int weights[] = {-500, -1500, -2500, -3500, -4500, -5500, 500, 1500, 2500, 3500, 4500, 5500};

const int MAX_CALIBRATION_SAMPLES = 250;
const long LARGE_ERROR_THRESHOLD = 3000;

// Pole pro kalibraci (zjistíme je během kalibrační fáze)
unsigned int sensorMin[NUM_SENSORS];
unsigned int sensorMax[NUM_SENSORS];

// Pole pro kalibrované hodnoty senzorů (0-1000)
int sensorValues[NUM_SENSORS];

// Nastavení průběhu kalibrace
const int intervalCalibration = 500; // čas kdy auto přejede čáru pro kalibraci z jedné strany na druhou
const int speedCalibration = 50;     // rychlost při kalibraci
const int countOverLine = 4;         // počet přejetí čáry při kalibraci
unsigned long actualTime;            // pro řízení fází kalibrace
unsigned long previousTime = 0;      // pro řízení fází kalibrace
int actualState = 0;                 // pro řízení fází kalibrace

// proměnné pro sbírání dat pro analýzu hodnot senzorů po kalibraci
int measuringCount = 0;                            // počet měření, maximální hodnota by měla být menší než hodnota druhé dimenze v předchozím poli
int sensors[NUM_SENSORS][MAX_CALIBRATION_SAMPLES]; // dvourozměrné pole hodnot ze všech senzorů získaných v průběhu kalibrace - odpovídá celkové době kalibrace, kolik stačí za tu dobu naměřit hodnot
uint16_t *sensors_u = nullptr;                     // optimalizované jednorozměrné pole
int rsensors[NUM_SENSORS];                         // neupravené hodnoty senzorů pro účel ladění v průběhu jízdy
int thresholds[NUM_SENSORS];                       // po analýze všech hodnot z předchozího pole, obsahuje hodnoty tresholdů = mezí pro každý senzor.
                                                   // jde v podstatě o to, že se v poli hodnot, najde největší mezera mezi dvěma nejbližšími hodnotami, které budou představovat černou a bílou a vypočítá se jejich průměr
int thresholds_u[NUM_SENSORS];
uint16_t sensorL, sensorR; // v průběhu jízdy vozítka aktuální bitovou mapu na levé a na pravé straně senzoru
                           // u každého senzoru podle aktuální změřené hodnoty a podle pole mezí zjistí, v daný čas, jestli jde o hodnotu 1 = je nad mezí
                           // nebo o hodnoru 0 = je pod mezí
                           // nejnižší bity jsou uprostřed senzoru a nejvyšší bity na krajích senzoru

uint16_t maskBackLine = 0b0000000000001111; // vymaskuje krajní bity, když chci zjistit, zda se blížíme k ostré zatáčce

const int timeToTurn90 = 450;  // čas pro pootočení o 90 stupňů
unsigned long turningTime = 0; // uloží aktuální čas před otáčením o 90 stupňů
bool isTurningLeft = false;    // stav otáčení vlevo o 90 stupňů
bool isTurningRight = false;   // stav otáčení vpravo o 90 stupňů

// proměnné pro jízdu ostrými zatáčkami
bool lineOnTheEdge = false;      // výsledek testu, zda je na levé nebo pravé straně vidět čára
bool startTurning = false;       // označení toho, že se vozítko právě otáčí v ostré zatáčce
unsigned int countBalancing = 0; // počítá množství sečtených hodnot pro balancing - při průjezdu ostrou zatáčkou, aby vypočetl aritmetický průměr
long sensorBalance = 0;          // vyhodnocení sečtených hodnot v průběhu dojezdu k ostré zatáčce (sensorL a sensorR) + nebo - podle toho podle toho čeho je víc
                                 // na základě toho se později rozhodne, kam zatáčet

const unsigned int limit90degrees = (1U << (NUM_SENSORS / 2)) - 1; // hodnota pro odhad 90 stupňové zatáčky
const unsigned int edgeBit = (int)(NUM_SENSORS / 2) - 1;           // pozice krajního senzoru
long emergencyTurnError = 0;                                       // v případě chyby při ostrých otáčkách

const int maxSpeed = 90;  // Maximální rychlost motorů (0-255)
const int baseSpeed = 60; // Základní rychlost, když je chyba 0

// Výpočet průměrné rychlosti motorů z posledních několika měření
int averageSpeedLMotor = 0;
int averageSpeedRMotor = 0;
float averageSpeedLMotor_f = 0.0;
float averageSpeedRMotor_f = 0.0;
const float alpha = 0.1; // Faktor vyhlazování (0.0 - 1.0)

// PID Konstanty
const double Kp = 0.0072;
const double Ki = 0.0;
const double Kd = 0.09;

// PID Proměnné
double error = 0;
double lastError = 0;
double integral = 0;
double derivative = 0;

// Funkce pro analýzu dat po jejich získání a kalibraci
void analyzeData();

// Funkce pro řazení pole od nejmenšího po největší
void countingSort(int inputArray[], int outputArray[], int size, int min, int max);
void countingSort_u(uint16_t *inputArray, uint16_t *outputArray, int size, int min, int max, unsigned int sensor);

// Funkce pro výpočet chyby (pozice čáry)
void calculateError();

// Funkce pro kalibraci vozítka: získá data a u každého senzoru zjistí jeho minimum a maximum
void calibrateVehicle();

// Funkce která získává kalibrační čísla - meze - thresholdy pro každý senzor v průběhu analýzy získaných hodnot po kalibraci
int getCalNumber(int hodnoty[], int velikost, unsigned int min, unsigned int max);
int getCalNumber_u(uint16_t *hodnoty, int velikost, unsigned int min, unsigned int max, unsigned int sensor);

// Funkce pro tisk Byte jako řadu bitů - ve dvojkové soustavě
void printByteAsBinary(byte b);

// Funkce pro čtení a normalizaci hodnot ze senzorů - v průběhu jízdy
void readSensors();

void setup()
{
  // nastavení sériové komunikace
  Serial.begin(115200);
  delay(1000);

  sensors_u = new (std::nothrow) uint16_t[NUM_SENSORS * MAX_CALIBRATION_SAMPLES];

  // začíná kalibrace - získává data a hledá jejich minima a maxima
  calibrateVehicle();

  // začíná analýza dat - vytváří thresholdy
  analyzeData();

  // uvolnění pole pro kalibraci
  delete[] sensors_u;
  sensors_u = nullptr;
}

void loop()
{
  // 1. Načíst a namapovat hodnoty senzorů
  readSensors();

  // 2. Reakce na čtení binární mapy senzorů
  // STAV 1: Právě probíhá 90° otočka? (Nejvyšší priorita)
  if (isTurningLeft)
  {
    if ((millis() - turningTime) < timeToTurn90)
    {
      // Otáčení stále probíhá
      if (motorsOn)
      {
        motorL.setSpeed(-baseSpeed);
        motorR.setSpeed(baseSpeed);
      }
    }
    else
    {
      // Otáčení právě skončilo, resetujeme stav
      isTurningLeft = false;
    }
  }
  else if (isTurningRight)
  {
    if ((millis() - turningTime) < timeToTurn90)
    {
      // Otáčení stále probíhá
      if (motorsOn)
      {
        motorL.setSpeed(baseSpeed);
        motorR.setSpeed(-baseSpeed);
      }
    }
    else
    {
      // Otáčení právě skončilo, resetujeme stav
      isTurningRight = false;
    }
  }
  else if (startTurning)
  {
    // STAV 2: Právě probíhá otáčka k nalezení zpětné čáry ostré zatáčky
    if ((sensorL > 0) || (sensorR > 0))
    {
      // Čára nalezena, ukončíme celý manévr
      lineOnTheEdge = false;
      startTurning = false;
      sensorBalance = 0;
      countBalancing = 0;
    }
  }
  else if (lineOnTheEdge)
  {
    // STAV 3: Jsme ve stavu "čára na hraně pole senzorů", ale ještě jsme se nezačali otáčet
    if (sensorL == 0 && sensorR == 0)
    {
      // Čára zmizela, zahajujeme otočku
      startTurning = true;
      // Podle odhadu inklinace ostré zatáčky rozhodneme směr otáčení
      long endOfTurn = (countBalancing > 0) ? (sensorBalance / countBalancing) : emergencyTurnError;

      // pokud bude záporný, otáčí vlevo, jinak vpravo
      if (endOfTurn < 0)
      {
        // začne zatáčet vlevo
        if (motorsOn)
        {
          motorL.setSpeed(-baseSpeed);
          motorR.setSpeed(baseSpeed);
        }
      }
      else
      {
        // začne zatáčet vpravo
        if (motorsOn)
        {
          motorL.setSpeed(baseSpeed);
          motorR.setSpeed(-baseSpeed);
        }
      }
    }
    else
    {
      // Stále vidíme čáry, sbíráme data a jedeme "naslepo"
      sensorBalance = sensorBalance - sensorL + sensorR;
      countBalancing++;
      if (motorsOn)
      {
        motorL.setSpeed(averageSpeedLMotor);
        motorR.setSpeed(averageSpeedRMotor);
      }
    }
  }
  else
  {
    // Zde jsme v normálním režimu jízdy PID regulátor a hledáme spouštěče
    // detekce nové 90° zatáčky vlevo
    if ((sensorL >= limit90degrees) && (sensorR <= 1))
    {
      isTurningLeft = true;
      if (debugOn)
        Serial.println("***TURNING LEFT***");
      turningTime = millis();

      // detekce nové 90° zatáčky vpravo
    }
    else if ((sensorR >= limit90degrees) && (sensorL <= 1))
    {
      isTurningRight = true;
      if (debugOn)
        Serial.println("***TURNING RIGHT***");
      turningTime = millis();

      // detekce nové ostré zatáčky
    }
    else
    {
      // Uděláme "snímek" aktuální chyby - Vypočítá chybu (pozici čáry)
      calculateError();

      bool edgeIsActive = ((((sensorL >> edgeBit) & 0x01) == 1) && ((sensorR & maskBackLine) > 0) && ((sensorR & maskBackLine) < 4)) || ((((sensorR >> edgeBit) & 0x01) == 1) && ((sensorL & maskBackLine) > 0) && ((sensorL & maskBackLine) < 4));

      if (edgeIsActive)
      {
        lineOnTheEdge = true;
        if (debugOn)
          Serial.println("***LINE ON THE EDGE***");
        emergencyTurnError = error; // Uložíme si ji pro případ nouze

        // Pokud žádná událost nenastala, pokračujeme dál s PID
      }
      else
      {

        if (debugOn)
          Serial.print("***PID***: ");
        if (debugOn)
          Serial.println(error);
        // Výpočet PID korekce
        // účel integrační složky je vyrovnávat postupné výchylky v průběhu delší dráhy beze změn, ale dráha je z principu plná změn, tak se většinou nevyplatí ji využít
        integral += error;
        // Omezení integrační složky (anti-windup) stejně se nepoužije: Ki = 0 jen pro dlouhé rovné úseky...
        integral = constrain(integral, -20000, 20000);

        derivative = error - lastError;

        double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Výpočet rychlosti motorů
        int leftMotorSpeed = baseSpeed + correction;
        int rightMotorSpeed = baseSpeed - correction;

        // Omezení rychlosti motorů
        leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
        rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

        // Nastavení motorů
        if (motorsOn)
        {
          motorL.setSpeed(leftMotorSpeed);
          motorR.setSpeed(rightMotorSpeed);
        }

        // Výpočet klouzavého průměru rychlosti motorů - pro jízdu na slepo
        averageSpeedLMotor_f = (alpha * leftMotorSpeed) + ((1.0 - alpha) * averageSpeedLMotor_f);
        averageSpeedRMotor_f = (alpha * rightMotorSpeed) + ((1.0 - alpha) * averageSpeedRMotor_f);

        averageSpeedLMotor = (int)averageSpeedLMotor_f;
        averageSpeedRMotor = (int)averageSpeedRMotor_f;

        // Uložení chyby pro další iteraci
        lastError = error;
      }
    }
  }
}

void countingSort_u(uint16_t *inputArray, uint16_t *outputArray, int size, int min, int max, unsigned int sensor)
{
  if (size == 0)
    return; // Nic k třídění

  uint16_t countArraySize = max - min + 1;
  uint16_t *countArray = new (std::nothrow) uint16_t[countArraySize];
  if (countArray == nullptr)
  {
    Serial.println("Chyba: Nedostatek pameti pro alokaci countArray!");
    return;
  }

  // Vynulování pomocného pole
  memset(countArray, 0, countArraySize * sizeof(uint16_t));

  // Spočítání výskytů každého prvku
  for (uint16_t i = 0; i < size; i++)
    countArray[inputArray[i * NUM_SENSORS + sensor] - min]++;

  // Výpočet kumulativních součtů.
  for (uint16_t i = 1; i < countArraySize; i++)
    countArray[i] += countArray[i - 1];

  // Sestavení výstupního pole.
  for (uint16_t i = size - 1; i >= 0; i--)
  {
    uint16_t value = inputArray[i * NUM_SENSORS + sensor];
    uint16_t position = countArray[value - min] - 1;
    outputArray[position * NUM_SENSORS + sensor] = value;
    countArray[value - min]--;
  }

  // Uvolnění alokované paměti, abychom předešli memory leaku.
  delete[] countArray;
}

void countingSort(int inputArray[], int outputArray[], int size, int min, int max)
{

  if (size == 0)
    return; // Nic k třídění

  int countArraySize = max - min + 1;
  int *countArray = new int[countArraySize];
  if (countArray == nullptr)
  {
    Serial.println("Chyba: Nedostatek pameti pro alokaci countArray!");
    return;
  }

  // Vynulování pomocného pole
  // for (int i = 0; i < countArraySize; i++) countArray[i] = 0;
  memset(countArray, 0, countArraySize * sizeof(int));

  // Spočítání výskytů každého prvku
  for (int i = 0; i < size; i++)
    countArray[inputArray[i] - min]++;

  // Výpočet kumulativních součtů.
  for (int i = 1; i < countArraySize; i++)
    countArray[i] += countArray[i - 1];

  // Sestavení výstupního pole.
  for (int i = size - 1; i >= 0; i--)
  {
    int value = inputArray[i];
    int position = countArray[value - min] - 1;
    outputArray[position] = value;
    countArray[value - min]--;
  }

  // Uvolnění alokované paměti, abychom předešli memory leaku.
  delete[] countArray;
}

int getCalNumber_u(uint16_t *hodnoty, int velikost, unsigned int min, unsigned int max, unsigned int sensor)
{
  uint16_t maxDiff = 0;
  uint16_t gapStartIndex = -1;
  uint16_t lowerBound;
  uint16_t upperBound;
  uint16_t *sortedHodnoty = new (std::nothrow) uint16_t[velikost];

  if (sortedHodnoty == nullptr)
  {
    Serial.println("Chyba: Nedostatek pameti pro alokaci countArray!");
    return 0;
  }

  // řazení celého pole od nejmenšího po největší
  memset(sortedHodnoty, 0, velikost * sizeof(uint16_t));
  countingSort_u(hodnoty, sortedHodnoty, velikost, min, max, sensor);

  if (debugAnalysis)
  {
    for (uint16_t i = 0; i < velikost - 1; i++)
      Serial.printf("%d, ", sortedHodnoty[i * NUM_SENSORS + sensor]);
    Serial.println();
  }

  // po seřazení se snadněji určí největší mezera mezi daty, uprostřed které se pak stanoví mezní hodnota
  for (uint16_t i = 0; i < velikost - 1; i++)
  {
    uint16_t currentDiff = sortedHodnoty[(i + 1) * NUM_SENSORS + sensor] - sortedHodnoty[i * NUM_SENSORS + sensor];
    // hledá největší vzdálenost mezi dvěma hodnotama
    if (currentDiff > maxDiff)
    {
      maxDiff = currentDiff;
      gapStartIndex = i;
    }
  }

  // hledání horní a dolní meze v největší mezeře mezi daty
  if (gapStartIndex != -1)
  {
    // Spodní mezí bude hodnota hned za prvním seskupením
    lowerBound = sortedHodnoty[gapStartIndex * NUM_SENSORS + sensor];
    // Horní mezí bude hodnota hned před druhým seskupením
    upperBound = sortedHodnoty[(gapStartIndex + 1) * NUM_SENSORS + sensor];
  }
  else
  {
    // Prázdný interval nenalezen (možná všechny hodnoty tvoří jednu skupinu)
    lowerBound = -1;
    upperBound = -1;
  }

  // Vypočte hodnotu mezi dvěma okraji mezery v datech
  return (lowerBound + (int)(maxDiff / 2));
}

int getCalNumber(int hodnoty[], int velikost, unsigned int min, unsigned int max)
{
  int maxDiff = 0;
  int gapStartIndex = -1;
  int lowerBound;
  int upperBound;
  int sortedHodnoty[velikost];

  // řazení celého pole od nejmenšího po největší
  memset(sortedHodnoty, 0, velikost * sizeof(int));
  countingSort(hodnoty, sortedHodnoty, velikost, min, max);

  if (debugAnalysis)
  {
    for (int i = 0; i < velikost - 1; i++)
      Serial.printf("%d, ", sortedHodnoty[i]);
    Serial.println();
  }

  // po seřazení se snadněji určí největší mezera mezi daty, uprostřed které se pak stanoví mezní hodnota
  for (int i = 0; i < velikost - 1; i++)
  {
    int currentDiff = sortedHodnoty[i + 1] - sortedHodnoty[i];
    // hledá největší vzdálenost mezi dvěma hodnotama
    if (currentDiff > maxDiff)
    {
      maxDiff = currentDiff;
      gapStartIndex = i;
    }
  }

  // hledání horní a dolní meze v největší mezeře mezi daty
  if (gapStartIndex != -1)
  {
    // Spodní mezí bude hodnota hned za prvním seskupením
    lowerBound = sortedHodnoty[gapStartIndex];
    // Horní mezí bude hodnota hned před druhým seskupením
    upperBound = sortedHodnoty[gapStartIndex + 1];
  }
  else
  {
    // Prázdný interval nenalezen (možná všechny hodnoty tvoří jednu skupinu)
    lowerBound = -1;
    upperBound = -1;
  }

  // Vypočte hodnotu mezi dvěma okraji mezery v datech
  return (lowerBound + (int)(maxDiff / 2));
}

void printBits(uint16_t value, int numBits)
{
  for (int i = numBits - 1; i >= 0; i--)
  {
    Serial.print(bitRead(value, i));
  }
}

void readSensors()
{
  // projede všechny senzory
  sensorL = 0;
  sensorR = 0;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    // u každého senzoru odečte jeho aktuální hodnotu
    int rawValue = analogRead(sensorPins[i]);

    // vypočte bitovou mapu senzoru na každé straně
    if (i < NUM_SENSORS / 2)
    {
      if (rawValue > thresholds_u[i])
        sensorL |= (1 << i);
    }
    else
    {
      int j = i - (NUM_SENSORS / 2);
      if (rawValue > thresholds_u[i])
        sensorR |= (1 << j);
    }

    // uloží čtené hodnoty do pole pro pozdější tisk pro debug
    rsensors[i] = rawValue;

    // map() převede hodnotu z kalibrovaného rozsahu na škálu 0-1000
    sensorValues[i] = map(rawValue, sensorMin[i], sensorMax[i], 0, 1000);
    // Omezení hodnot ještě pro jistotu
    sensorValues[i] = constrain(sensorValues[i], 0, 1000);
  }

  // debug tisk - stavu mapy senzorů
  if (debugResultsAnalysis)
  {
    // tisk stavu senzorů
    printBits(sensorL, NUM_SENSORS / 2);
    Serial.print("|");
    printBits(sensorR, NUM_SENSORS / 2);
    Serial.print("  ---  ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.printf("%04d - ", rsensors[i]);
    }
    Serial.println();
  }
}

void calculateError()
{
  long weightedSum = 0;
  long sumOfValues = 0;
  bool lineSeen = false;

  // počítá se aritmetický průměr hodnot senzorů a nikoli součet hodnot senzorů -> menší číslo
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    // zjistíme, jestli vidí čáru
    if (rsensors[i] > thresholds_u[i])
      lineSeen = true;
    // počítám položky váženého průměru
    weightedSum += (long)sensorValues[i] * weights[i];
    sumOfValues += sensorValues[i];
  }

  if (!lineSeen)
  {
    // Pokud nevidíme čáru, použijeme poslední známou chybu
    // Pokud byla poslední chyba velká (robot byl daleko), otočíme se na místě
    if (lastError > LARGE_ERROR_THRESHOLD)
    {
      error = (NUM_SENSORS - 1) * 500;
    }
    else if (lastError < -LARGE_ERROR_THRESHOLD)
    {
      error = (NUM_SENSORS - 1) * -500;
    }
    // Jinak `error` zůstává jako `lastError`, robot pojede rovně
  }
  else
  {
    if (sumOfValues > 0)
    {
      // Vypočítáme vážený průměr
      error = weightedSum / sumOfValues;
    }
    else
    {
      // Pokud je součet 0, nemůžeme dělit.
      error = 0;
    }
  }
}

void calibrateVehicle()
{
  // čeká než začne kalibrovat po položení na čáru a zapnutí
  delay(1000);
  // --- KALIBRAČNÍ FÁZE ---
  if (debugOn)
    Serial.println("***KALIBRACE START***");

  // Inicializace min a max hodnot v poli pro všechny senzory na minimálně a maximálně možné hodnoty
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  // pootočení o polovinu oblouku na jednu stranu čáry pro pozdější kalibraci
  if (motorsOn)
  {
    motorL.setSpeed(speedCalibration);
    motorR.setSpeed(-speedCalibration);
    delay((unsigned long)intervalCalibration / 2);
  }

  // nastavení počátečního času kalibrace, od kterého se bude měřit její čas
  unsigned long calibrationStartTime = millis();
  actualTime = calibrationStartTime;
  // Kalibrace po dobu odpovídající počtu přejetí čáry x čas na přejetí z jedné na druhou stranu
  while ((millis() - calibrationStartTime) < (intervalCalibration * countOverLine))
  {
    actualTime = millis();
    // stopování jednoho přejezdu ze strany na stranu
    if ((actualTime - previousTime) >= intervalCalibration)
    {
      previousTime = actualTime;
      // přepínání stavu
      if (actualState == 0)
      {
        // v jednom stavu jede na jednu stranu
        if (motorsOn)
        {
          motorL.setSpeed(-speedCalibration);
          motorR.setSpeed(speedCalibration);
        }
        if (debugCalibration)
          Serial.print("Jeden směr\n");
        actualState = 1;
      }
      else
      {
        // v druhém stavu na druhou
        if (motorsOn)
        {
          motorL.setSpeed(speedCalibration);
          motorR.setSpeed(-speedCalibration);
        }
        if (debugCalibration)
          Serial.print("Druhý směr\n");
        actualState = 0;
      }
    }
    if (measuringCount < MAX_CALIBRATION_SAMPLES)
    {
      // v průběhu kalibrace hledá maxima a minima pro každý senzor
      for (int i = 0; i < NUM_SENSORS; i++)
      {
        int hodnota = analogRead(sensorPins[i]);
        // celou dobu kalibrace se ukládají hodnoty všech senzorů
        sensors[i][measuringCount] = hodnota;
        sensors_u[measuringCount * NUM_SENSORS + i] = (uint16_t)hodnota;

        if (hodnota < sensorMin[i])
          sensorMin[i] = hodnota;
        if (hodnota > sensorMax[i])
          sensorMax[i] = hodnota;
        if (debugCalibration)
          Serial.printf("%d - %d - %d : ", sensorMin[i], sensors[i][measuringCount], sensorMax[i]);
        if (debugCalibration)
          Serial.printf("%d - %d - %d : ", sensorMin[i], sensors_u[measuringCount * NUM_SENSORS + i], sensorMax[i]);
      }
      Serial.println();
      measuringCount++;
    }
    // krátká pauza v měření
    delay(10);
  }

  // vozítko se vrátí zpět na čáru
  if (motorsOn)
  {
    motorL.setSpeed(-speedCalibration);
    motorR.setSpeed(speedCalibration);
  }
  delay((unsigned long)intervalCalibration / 2);

  // než vozítko vyjede, vypnou se motory
  motorL.setSpeed(0);
  motorR.setSpeed(0);

  if (debugOn)
    Serial.println("***KALIBRACE end***");
}

void analyzeData()
{
  // začíná analýza nabíraných dat
  if (debugOn)
    Serial.println("***ANALÝZA DAT start***");

  // vypočte pro každý senzor mez, nad co je to 1 a pod co je to 0
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    thresholds[i] = getCalNumber(sensors[i], measuringCount, sensorMin[i], sensorMax[i]);
    thresholds_u[i] = getCalNumber_u(sensors_u, measuringCount, sensorMin[i], sensorMax[i], i);
    if (debugAnalysis)
      Serial.printf("%d\n", thresholds[i]);
    if (debugAnalysis)
      Serial.printf("%d\n", thresholds_u[i]);
  }

  if (debugOn)
    Serial.println("***ANALÝZA DAT end***");
}
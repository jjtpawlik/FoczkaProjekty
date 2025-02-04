#include <math.h>

// Definicje pinów
const int thermistorPin = A1; // Pin analogowy dla termistora
const int relayPin = 9;       // Pin cyfrowy dla przekaźnika

// Parametry termistora
const float nominalResistance = 10200; // Nominalna rezystancja termistora w 25°C (10kOhm)
const float nominalTemperature = 25.0; // Nominalna temperatura w °C
const float bCoefficient = 3950;       // Współczynnik Beta
const float seriesResistor = 10000;    // Rezystor szeregowy (10kOhm)

// Zmienne globalne
float docelowaTemperatura = 150.0 ; // Temperatura docelowa w °C
double kp = 2.0;                 // Współczynnik proporcjonalny
double ki = 5.0;                 // Współczynnik całkujący
double kd = 1.0;                 // Współczynnik różniczkujący

// Zmienne PID
double setpoint = docelowaTemperatura;
double input;           // Aktualna temperatura (odczytana z termistora)
double output;          // Wyjście PID (używane do sterowania przekaźnikiem)

// Parametry PID
double integral = 0, previousError = 0;

void setup() {
  Serial.begin(9600);

  // Ustawienie pinu dla przekaźnika
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Wyłącz grzałkę na starcie
    
  // Ustawienie pinu dla termistora
  pinMode(thermistorPin, INPUT);
}

void loop() {
  // Odczytaj temperaturę z termistora
  input = readTemperature();

  // Wykonaj obliczenia PID
  output = computePID(setpoint, input);

  // Steruj przekaźnikiem w zależności od wyjścia PID
  if (output > 128) {
    digitalWrite(relayPin, LOW); // Włącz grzałkę
  } else {
    digitalWrite(relayPin, HIGH); // Wyłącz grzałkę
  }

// Aktualny czas (np. symulacja czasu w sekundach)
  unsigned long czas = millis() / 1000;
  // Debugowanie
  //Serial.print("ADC ");
  //Serial.print((float)analogRead(thermistorPin));
  //Serial.print(" ");
  //Serial.print("Czas: ");
  Serial.print(czas);
  Serial.print(",");
 // Serial.print("Temperatura: ");
  Serial.println(input);
  
  //Serial.print(" °C");
  //Serial.print(" | Wyjście PID: ");
  //Serial.println(output);

  delay(1000); // Odczekaj 1 sekundę
}

// Funkcja do obliczania temperatury z termistora
float readTemperature() {
  int adcValue = analogRead(thermistorPin);
  float resistance = (1023.0 / (float)adcValue - 1.0) * seriesResistor;

  // Oblicz temperaturę w kelwinach
  float temperatureK = 1.0 / (log(resistance / nominalResistance) / bCoefficient + 1.0 / (nominalTemperature + 273.15));

  // Konwertuj na stopnie Celsjusza
  return temperatureK - 273.15;
}

// Funkcja PID
float computePID(float setpoint, float input) {
  double error = setpoint - input;
  integral += error;
  double derivative = error - previousError;
  output = kp * error + ki * integral + kd * derivative;
  output = constrain(output, 0, 255); // Ogranicz wyjście do zakresu 0-255
  previousError = error;
  return output;
}

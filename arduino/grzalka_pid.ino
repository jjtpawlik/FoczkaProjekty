#include <math.h>

// Definicje pinów
const int thermistorPin = A1; // Pin analogowy dla termistora
const int relayPin = 9;       // Pin cyfrowy dla przekaźnika
const int mosfetPin = 10;     // Pin PWM dla mosfetu

// Parametry termistora
const float nominalResistance = 10200; // Nominalna rezystancja termistora w 25°C (10kOhm)
const float nominalTemperature = 25.0; // Nominalna temperatura w °C
const float bCoefficient = 3950;       // Współczynnik Beta
const float seriesResistor = 10000;    // Rezystor szeregowy (10kOhm)

// Zmienne globalne
float docelowaTemperatura = 50.0 ; // Temperatura docelowa w °C
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

  pinMode(relayPin, OUTPUT);    // Ustawienie pinu dla przekaźnika
  pinMode(thermistorPin, INPUT);// Ustawienie pinu dla termistora
  pinMode(mosfetPin, OUTPUT);   // Ustawienie pinu dla mosfetu

  digitalWrite(relayPin, HIGH); // Wyłącz grzałkę na starcie 
}

void loop() {
  // Odczytaj temperaturę z termistora
  input = readTemperature();

  // Wykonaj obliczenia PID
  output = computePID(setpoint, input);

  // Steruj przekaźnikiem i mosfetem w zależności od wyjścia PID
  if (output > 128) {
    digitalWrite(relayPin, LOW);    // Włącz przekaźnik (załączany niskim sygnałem)
    analogWrite(mosfetPin, 255);    // Włącz mosfet na pełną moc
  } else {
    digitalWrite(relayPin, HIGH);   // Wyłącz przekaźnik
    analogWrite(mosfetPin, 0);    // Wyłącz mosfet
  }

  // Aktualny czas (np. symulacja czasu w sekundach)
  unsigned long czas = millis() / 1000; //czas probkowania: 1s
  // Debugowanie
  Serial.print("Czas: |");
  Serial.print(czas);
  Serial.print("|");
  Serial.print("Temperatura: |");
  Serial.print(input);
  Serial.print("|");
  Serial.print("Wyjście PID: |");
  Serial.print(output);
  Serial.println("|");
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
  //Debugowanie:
  Serial.print("Error PID: |");
  Serial.print(error);
  Serial.print("|");
  Serial.print("Integral: |");
  Serial.print(integral);
  Serial.print("|");
  return output;
}

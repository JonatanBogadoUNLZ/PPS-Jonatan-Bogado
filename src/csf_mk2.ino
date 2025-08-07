#include <SPI.h>              // Para comunicación SPI con el display y la SD
#include <Adafruit_GFX.h>     // Librería gráfica base para el display
#include <Adafruit_ILI9341.h> // Driver específico para tu display ILI9341
#include <ESP32Encoder.h>     // Para el encoder rotatorio KY-040 del menú
#include <SD.h>               // Para la tarjeta SD
#include <AccelStepper.h>     // Para el control de los motores paso a paso
#include <Wire.h>             // Opcional, si usas algún sensor I2C en el futuro
// #include <WiFi.h>          // Descomentar si implementas la pantalla de WiFi
// #include <EEPROM.h>        // Opcional, para almacenar configuraciones persistentes sin SD
#include <Bounce2.h> 

// Pines para el display ILI9341 (ejemplo, ajusta según tu conexión)
#define TFT_CS 15 
#define TFT_DC 2 
#define TFT_RST 4 
// SPI Pines para ILI9341 - Estos son los pines SPI "por defecto" del ESP32
// que se usan con la librería Adafruit_ILI9341. 
// No necesitas definirlos explícitamente a menos que uses SPI personalizado. 
// Si tu display usa los pines físicos 18 y 23 para SCK y MOSI, Adafruit_ILI9341 
// los utilizará automáticamente con la interfaz SPI del ESP32. 
// #define SPI_SCK 18 // No es necesario definir si se usan los default 
// #define SPI_MOSI 23 // No es necesario definir si se usan los default 
// #define SPI_MISO 19 // Generalmente el MISO no se usa para ILI9341 Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Encoder de medicion de perfil
#define PROFILE_ENCODER_A 25
#define PROFILE_ENCODER_B 16
ESP32Encoder profileEncoder;

// Encoder del menu de usuario. Pines para el encoder rotatorio del menú (KY-040)
#define ENCODER_CLK_PIN 34
#define ENCODER_DT_PIN  35
#define ENCODER_SW_PIN  33 // Pulsador del encoder
ESP32Encoder menuEncoder; // Objeto para el encoder de menú
Bounce menuButtonDebouncer = Bounce(); // Objeto Bounce para el botón del encoder

#define BTN_MENU 4
Bounce menuScreenButtonDebouncer = Bounce();

// Pines para los motores paso a paso (NEMA) - Ejemplo para 3 motores con drivers DIR/STEP
// Ajusta los pines de Enable según tu driver.
#define DIR_MOTOR_1 26 // Motor de Tracción 1
#define STEP_MOTOR_1 19
#define DIR_MOTOR_2 21 // Motor de Tracción 2
#define STEP_MOTOR_2 22
#define DIR_MOTOR_3 32 // Motor de Tracción 3 (salida)
#define STEP_MOTOR_3 5
#define ENABLE_MOTORS_PIN 17 // Pin para activar/desactivar todos los drivers (si es común)

// Crear objetos AccelStepper para los motores
AccelStepper motor1(AccelStepper::DRIVER, STEP_MOTOR_1, DIR_MOTOR_1);
AccelStepper motor2(AccelStepper::DRIVER, STEP_MOTOR_2, DIR_MOTOR_2);
AccelStepper motor3(AccelStepper::DRIVER, STEP_MOTOR_3, DIR_MOTOR_3);

// Pines para el encoder del perfil
#define PROFILE_ENCODER_A 25
#define PROFILE_ENCODER_B 16
ESP32Encoder profileEncoder; // Objeto para el encoder del perfil
float stepsPerMeter = 6366.0; // ¡IMPORTANTE: Calibrar este valor! Pasos por metro del encoder del perfil.

// Pines para los sensores inductivos
#define SENSOR_IND_1 36 // Sensor más cercano a la entrada
#define SENSOR_IND_2 39 // Sensor más cercano a la matriz de corte (dentro del offset)

// Pines de botones
#define BTN_START 27 // Pulsador de inicio
#define BTN_EMERGENCY 13 // Pulsador de parada de emergencia
Bounce startButtonDebouncer = Bounce();
Bounce emergencyButtonDebouncer = Bounce();

// Pin para la cizalla hidráulica (ejemplo, conectado a un relé)
#define CIZALLA_PIN 12

// Pin para el sensor de fin de carrera del pistón hidráulico (ejemplo)
#define CIZALLA_RETRAIDA_SENSOR_PIN 30

// Pin para el chip select de la tarjeta SD
#define SD_CS 14

// Definiciones de estado de la máquina
enum MachineState {
  IDLE,
  WAITING_FOR_PROFILE, // Esperando que el perfil sea detectado por el primer sensor
  MOVING_TO_OFFSET,    // Moviendo el perfil hasta el punto de calibración (ambos sensores activos)
  READY_TO_CUT,        // Perfil en posición inicial para el primer corte
  PERFORMING_CUT_CYCLE, // Ciclo de corte activo
  PAUSED,
  EMERGENCY_STOP,
  ERROR
};
MachineState currentState = IDLE;

// Estructuras de datos para los perfiles
enum ProfileType { PGC, PGU };
String profileTypeStrings[] = {"PGC", "PGU"};
int profileWidths[] = {60, 70, 90, 100, 140, 150, 200, 250}; // Anchos disponibles

struct ProfileSettings {
  ProfileType type;
  int widthIndex; // Índice en el array profileWidths
};

// Variables para la configuración actual
ProfileSettings currentProfileSettings = {PGC, 2}; // Perfil inicial: PGC 90 (índice 2)
float currentLength = 6.00;     // Largo total del perfil en metros
float currentCutLength = 1.00;  // Largo de corte en metros
int totalCuts = 0;              // Cantidad total de cortes a realizar para el perfil actual
int cutsRemaining = 0;          // Cortes restantes para el perfil actual

// Variables para el menú y display
int currentMenuOption = 0;
const int NUM_MAIN_OPTIONS = 5; // Perfil, Ancho, Largo, Largo Corte, Cantidad Cortes
int currentScreen = 0; // 0: Principal, 1: Datos Wifi, 2: Config
bool editingValue = false; // Indica si estamos editando un valor con el encoder

// Variables de offset para la cizalla
long matrixOffsetSteps = 0; // Offset en pasos del encoder desde la posición de los dos sensores hasta la cizalla

// Pin para el sensor de fin de carrera del pistón hidráulico (ejemplo) #define CIZALLA_RETRAIDA_SENSOR_PIN 30 // Ajusta este pin según tu conexión // Tiempos de retardo const unsigned long CUT_ACTIVATION_TIME_MS = 1000; // Tiempo que la cizalla está activa (bajando) const unsigned long PISTON_RETURN_TIMEOUT_MS = 5000; // Tiempo máximo de espera para que el pistón retorne


void drawHomeScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("--- MAQUINA DE CORTE ---");
  tft.println("");

  // Perfil
  if (currentMenuOption == 0 && currentScreen == 0 && !editingValue) tft.setTextColor(ILI9341_YELLOW); else tft.setTextColor(ILI9341_WHITE);
  tft.print("Perfil: ");
  tft.println(profileTypeStrings[currentProfileSettings.type]);

  // Ancho
  if (currentMenuOption == 1 && currentScreen == 0 && !editingValue) tft.setTextColor(ILI9341_YELLOW); else tft.setTextColor(ILI9341_WHITE);
  tft.print("Ancho: ");
  tft.println(profileWidths[currentProfileSettings.widthIndex]);

  // Largo
  if (currentMenuOption == 2 && currentScreen == 0 && !editingValue) tft.setTextColor(ILI9341_YELLOW); else tft.setTextColor(ILI9341_WHITE);
  tft.print("Largo: ");
  tft.print(currentLength, 2); tft.println(" m");

  // Largo Corte
  if (currentMenuOption == 3 && currentScreen == 0 && !editingValue) tft.setTextColor(ILI9341_YELLOW); else tft.setTextColor(ILI9341_WHITE);
  tft.print("Largo Corte: ");
  tft.print(currentCutLength, 2); tft.println(" m");

  // Cantidad de Cortes
  if (currentMenuOption == 4 && currentScreen == 0 && !editingValue) tft.setTextColor(ILI9341_YELLOW); else tft.setTextColor(ILI9341_WHITE);
  tft.print("Cortes Tot: "); tft.print(totalCuts);
  tft.print(" Rest: "); tft.println(cutsRemaining);

  tft.setCursor(0, tft.height() - 20);
  tft.setTextColor(ILI9341_CYAN);
  tft.print(" [OK] [MENU] [Corte]"); // Placeholder para botones físicos o táctiles
}

void drawWifiScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("--- DATOS WIFI ---");
  tft.println("");
  tft.println("SSID: TuRedWifi");
  tft.println("IP: 192.168.1.100");
  tft.println("");
  tft.println("Conectando..."); // O estado actual
  tft.setCursor(0, tft.height() - 20);
  tft.setTextColor(ILI9341_CYAN);
  tft.print(" [Volver]");
}

void drawConfigScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("--- CONFIGURACION ---");
  tft.println("");
  tft.println("Calibrar Motores");
  tft.println("Ver Historial");
  tft.println("Resetear Datos");
  tft.println("Calibrar Offset"); // Opción para calibrar el offset de la matriz
  tft.println("");
  tft.setCursor(0, tft.height() - 20);
  tft.setTextColor(ILI9341_CYAN);
  tft.print(" [Volver]");
}

void updateDisplay() {
  switch (currentScreen) {
    case 0: drawHomeScreen(); break;
    case 1: drawWifiScreen(); break;
    case 2: drawConfigScreen(); break;
    // Agrega más pantallas según sea necesario
  }
}

void handleMenuEncoder() {
  static long oldEncoderPos = 0;
  long newEncoderPos = menuEncoder.getCount();

  if (newEncoderPos != oldEncoderPos) {
    if (editingValue) {
      // Estamos editando un valor (Largo o Largo Corte)
      float step = 0.01; // Paso de 0.01 metros
      if (newEncoderPos > oldEncoderPos) {
        // Aumentar valor
        if (currentMenuOption == 2) currentLength += step;
        else if (currentMenuOption == 3) currentCutLength += step;
      } else {
        // Disminuir valor
        if (currentMenuOption == 2) currentLength -= step;
        else if (currentMenuOption == 3) currentCutLength -= step;
      }
      // Asegurarse de que los valores no sean negativos
      if (currentLength < 0) currentLength = 0;
      if (currentCutLength < 0) currentCutLength = 0;
      // Recalcular totalCuts cuando se cambian Largo o Largo Corte
      if (currentCutLength > 0) {
        totalCuts = floor(currentLength / currentCutLength);
        cutsRemaining = totalCuts;
      } else {
        totalCuts = 0;
        cutsRemaining = 0;
      }

    } else {
      // Estamos navegando en el menú
      if (newEncoderPos > oldEncoderPos) {
        currentMenuOption++;
      } else {
        currentMenuOption--;
      }
      // Limitar opciones del menú según la pantalla actual
      if (currentScreen == 0) { // Pantalla principal
        if (currentMenuOption >= NUM_MAIN_OPTIONS) currentMenuOption = 0;
        if (currentMenuOption < 0) currentMenuOption = NUM_MAIN_OPTIONS - 1;
      } else if (currentScreen == 1) { // Pantalla Wifi - no navegable
        currentMenuOption = 0;
      } else if (currentScreen == 2) { // Pantalla Config (ejemplo para 4 opciones)
        currentMenuOption %= 4;
        if (currentMenuOption < 0) currentMenuOption += 4;
      }
    }
    oldEncoderPos = newEncoderPos;
    updateDisplay();
  }
}

// Función para manejar el botón del encoder (seleccionar/entrar/salir edición)
void handleEncoderButton() {
  // Cuando se presiona el botón del encoder
  if (currentScreen == 0) { // Pantalla principal
    if (currentMenuOption == 0) { // Perfil
      if (!editingValue) {
        currentProfileSettings.type = (currentProfileSettings.type == PGC) ? PGU : PGC;
      }
    } else if (currentMenuOption == 1) { // Ancho
      if (!editingValue) {
        currentProfileSettings.widthIndex = (currentProfileSettings.widthIndex + 1) % (sizeof(profileWidths) / sizeof(profileWidths[0]));
      }
    } else if (currentMenuOption == 2 || currentMenuOption == 3) { // Largo o Largo Corte
      editingValue = !editingValue; // Alternar entre modo edición y navegación
      if (editingValue) {
        // Resetear el contador del encoder para empezar a editar desde 0
        menuEncoder.clearCount();
      }
    } else if (currentMenuOption == 4) { // Cantidad de Cortes (no editable directamente)
      // Podrías implementar una lógica para reiniciar el contador si es necesario.
    }
  } else if (currentScreen == 1 || currentScreen == 2) { // Pantallas Wifi o Configuración
    // Aquí puedes implementar la lógica de "Volver" o seleccionar opciones de configuración
    // Por simplicidad, un toque podría volver a la pantalla principal
    currentScreen = 0;
    currentMenuOption = 0; // Resetear opción al volver
  }
  updateDisplay();
}

// Función para cambiar de pantalla (ejemplo, un botón físico de "MENU")
void cycleScreens() {
  currentScreen = (currentScreen + 1) % 3; // 0:Principal, 1:Wifi, 2:Config
  currentMenuOption = 0; // Resetear la opción al cambiar de pantalla
  editingValue = false; // Asegurarse de que no estamos editando al cambiar de pantalla
  updateDisplay();
}


void setupMotors() {
  motor1.setMaxSpeed(1000.0);    // Ajusta la velocidad máxima según tus NEMA y driver
  motor1.setAcceleration(500.0); // Ajusta la aceleración
  motor2.setMaxSpeed(1000.0);
  motor2.setAcceleration(500.0);
  motor3.setMaxSpeed(1000.0);
  motor3.setAcceleration(500.0);

  pinMode(ENABLE_MOTORS_PIN, OUTPUT);
  digitalWrite(ENABLE_MOTORS_PIN, LOW); // Activar drivers (común para muchos drivers)
}

// Función para mover el perfil una distancia específica desde la posición actual del encoder del perfil
void moveProfileBy(float distance_m) {
  long targetSteps = (long)(distance_m * stepsPerMeter);
  long currentSteps = profileEncoder.getCount(); // Tomar la lectura actual como referencia

  motor1.moveTo(currentSteps + targetSteps);
  motor2.moveTo(currentSteps + targetSteps);
  motor3.moveTo(currentSteps + targetSteps);

  Serial.print("Moviendo perfil a ");
  Serial.print(distance_m);
  Serial.println(" metros.");

  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
    // Opcional: Actualizar el display con el avance si es una pantalla de progreso
  }
  Serial.println("Movimiento completado.");
}

// Función para realizar el corte
void performCut() {
  Serial.println("Realizando corte...");
  // 1. Detener motores de tracción
  motor1.stop(); motor2.stop(); motor3.stop();
  while(motor1.isRunning() || motor2.isRunning() || motor3.isRunning()){
    motor1.run(); motor2.run(); motor3.run();
  }

  // 2. Ejecutar pistón hidráulico (activar relé para bajar la cizalla)
  digitalWrite(CIZALLA_PIN, HIGH); // Cizalla BAJA
  Serial.println("Cizalla bajando...");
  delay(CUT_ACTIVATION_TIME_MS); // Espera el tiempo de bajada y corte

  // 3. Desactivar el pistón para que retorne a su posición inicial
  digitalWrite(CIZALLA_PIN, LOW); // Cizalla SUBE (se desactiva la presión para que retorne)
  Serial.println("Cizalla subiendo...");

  // 4. Esperar la lectura del sensor de fin de carrera de retorno (pistón arriba)
  unsigned long startTime = millis();
  bool pistonRetracted = false;
  while (millis() - startTime < PISTON_RETURN_TIMEOUT_MS) {
    if (digitalRead(CIZALLA_RETRAIDA_SENSOR_PIN) == HIGH) { // Asume HIGH cuando el pistón está arriba
      pistonRetracted = true;
      break; // El pistón ha retornado
    }
    delay(10); // Pequeña espera para no saturar el CPU
  }

  if (!pistonRetracted) {
    handleError("Fallo: El piston de la cizalla no retorno a su posicion inicial.");
    return; // Detiene el ciclo si hay un error
  }
  Serial.println("Cizalla en posicion inicial (arriba).");

  // 5. Registrar el corte en la SD
  logCut(currentProfileSettings.type, profileWidths[currentProfileSettings.widthIndex], currentCutLength);

  // 6. Decrementar cortes restantes
  if (cutsRemaining > 0) {
    cutsRemaining--;
  }
  updateDisplay(); // Actualizar la pantalla con los cortes restantes
  Serial.println("Corte completado.");
}


// Función para verificar si ambos sensores detectan el perfil
bool bothSensorsActive() {
  return (digitalRead(SENSOR_IND_1) == HIGH && digitalRead(SENSOR_IND_2) == HIGH); // O LOW, según tus sensores
}

// Función para verificar si el primer sensor detecta el perfil
bool firstSensorActive() {
  return (digitalRead(SENSOR_IND_1) == HIGH); // O LOW, según tu sensor
}

// Función de calibración del offset de la matriz (se llamaría desde el menú de Configuración)
void calibrateMatrixOffset() {
  Serial.println("Iniciando calibracion de offset. Coloque un perfil y presione Start.");
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0,0); tft.setTextColor(ILI9341_WHITE); tft.println("Calibrando Offset...");
  tft.println("Mueva perfil manualmente hasta que ambos sensores esten activos.");
  tft.println("Luego presione Start.");

  while (!bothSensorsActive()) {
    // Esperar a que el usuario mueva el perfil manualmente hasta la posición de calibración
    // Opcional: mostrar estado de los sensores en el display
    if (startButtonDebouncer.fell()) { // Si se presiona Start durante la calibración
        Serial.println("Detectado Start button durante calibracion");
        break; // Sale del bucle, asume que el perfil está posicionado
    }
    delay(50);
  }

  // En este punto, ambos sensores están activos, el perfil está en la posición de referencia
  matrixOffsetSteps = profileEncoder.getCount(); // Guarda el conteo del encoder
  Serial.print("Offset de matriz calibrado: ");
  Serial.print(matrixOffsetSteps);
  Serial.println(" pasos.");
  logCalibration("Matrix Offset:" + String(matrixOffsetSteps));

  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0,0); tft.setTextColor(ILI9341_WHITE); tft.println("Calibracion OK!");
  tft.print("Offset: "); tft.print(matrixOffsetSteps); tft.println(" pasos");
  delay(2000);
  updateDisplay(); // Volver a la pantalla principal
}


void initSDCard() {
  if (!SD.begin(SD_CS)) {
    Serial.println("Fallo al inicializar la tarjeta SD!");
    // Manejar el error, quizás mostrar en el display
    return;
  } else {
    Serial.println("Tarjeta SD inicializada.");
  }
}

void logData(String filename, String data) {
  File dataFile = SD.open(filename, FILE_APPEND);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
    Serial.print("Datos guardados en ");
    Serial.println(filename);
  } else {
    Serial.print("Error al abrir ");
    Serial.print(filename);
    Serial.println(" para escritura.");
  }
}

void logMachineState(String state) {
  String logEntry = String(millis()) + "," + state;
  logData("/machine_state.txt", logEntry);
}

void logError(String errorMsg) {
  String logEntry = String(millis()) + "," + errorMsg;
  logData("/errors.txt", logEntry);
}

void logCalibration(String calibrationData) {
  String logEntry = String(millis()) + "," + calibrationData;
  logData("/calibrations.txt", logEntry);
}

void logCut(ProfileType type, int width, float length) {
  String logEntry = String(millis()) + "," + profileTypeStrings[type] + "," + String(width) + "," + String(length, 2);
  logData("/cuts_history.txt", logEntry);
}


void handleStartButton() {
  if (startButtonDebouncer.fell()) { // Solo si el botón acaba de ser presionado
    if (currentState == IDLE) {
      Serial.println("Inicio de proceso solicitado.");
      logMachineState("START_REQUESTED");
      // Reiniciar contador de cortes si se inició un nuevo proceso
      if (currentCutLength > 0) {
        totalCuts = floor(currentLength / currentCutLength);
        cutsRemaining = totalCuts;
      } else {
        totalCuts = 0;
        cutsRemaining = 0;
      }
      updateDisplay(); // Actualizar la pantalla con los nuevos conteos

      currentState = WAITING_FOR_PROFILE;
      Serial.println("Estado: Esperando Perfil.");
      tft.fillScreen(ILI9341_BLACK);
      tft.setCursor(0,0); tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
      tft.println("Esperando perfil...");
      tft.setTextSize(1);
      tft.println("Introduzca el perfil hasta el primer sensor.");

    } else if (currentState == PERFORMING_CUT_CYCLE) {
      // Si ya está cortando, el botón de inicio podría ser una pausa/reanudación
      // Por ahora, lo dejaremos para "inicio" solamente
      Serial.println("Maquina ya en proceso de corte.");
    } else if (currentState == EMERGENCY_STOP) {
      Serial.println("La maquina esta en parada de emergencia. Resetea primero.");
      // No hacer nada, requiere resetear la emergencia
    }
  }
}

void handleEmergencyButton() {
  if (emergencyButtonDebouncer.fell()) { // Solo si el botón acaba de ser presionado
    if (currentState != EMERGENCY_STOP) {
      currentState = EMERGENCY_STOP;
      Serial.println("!!! PARADA DE EMERGENCIA ACTIVADA !!!");
      logError("EMERGENCY_STOP_ACTIVATED");
      // Detener inmediatamente todos los motores
      motor1.stop(); motor2.stop(); motor3.stop(); // Detiene los movimientos actuales
      digitalWrite(ENABLE_MOTORS_PIN, HIGH); // Desactiva los drivers de los motores
      digitalWrite(CIZALLA_PIN, LOW); // Asegura que la cizalla esté apagada
      tft.fillScreen(ILI9341_RED);
      tft.setCursor(0, tft.height()/2 - 10);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(3);
      tft.println("EMERGENCIA");
      tft.setTextSize(2);
      tft.setCursor(0, tft.height()/2 + 20);
      tft.println("Presione Reset para continuar"); // Placeholder para un botón de reset físico
    }
  }
}

// Función para reiniciar la máquina desde la parada de emergencia
void resetEmergency() {
  if (currentState == EMERGENCY_STOP) {
    Serial.println("Reseteando de emergencia...");
    digitalWrite(ENABLE_MOTORS_PIN, LOW); // Reactiva los drivers
    currentState = IDLE;
    updateDisplay(); // Vuelve a la pantalla principal
    logMachineState("EMERGENCY_RESET");
  }
}

// Función para gestionar errores (simplificado, se puede expandir)
void handleError(String errorMsg) {
  Serial.print("ERROR: ");
  Serial.println(errorMsg);
  logError(errorMsg);
  currentState = ERROR;
  tft.fillScreen(ILI9341_RED);
  tft.setCursor(0,0); tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
  tft.println("ERROR CRITICO!");
  tft.println(errorMsg);
  tft.println("Reinicie la maquina.");
  // Considerar un loop infinito o un reinicio forzado en errores críticos.
  while(true){
    // Esperar reinicio manual o un botón de reset
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando maquina de corte...");

  // Configurar pines de IO
  pinMode(ENABLE_MOTORS_PIN, OUTPUT);
  pinMode(CIZALLA_PIN, OUTPUT);
  digitalWrite(CIZALLA_PIN, LOW); // Asegura que la cizalla está apagada al inicio

  // Configurar botones con Bounce2
  startButtonDebouncer.attach(BTN_START, INPUT_PULLUP);
  startButtonDebouncer.interval(25); // Antirebote de 25ms
  emergencyButtonDebouncer.attach(BTN_EMERGENCY, INPUT_PULLUP);
  emergencyButtonDebouncer.interval(25);
  menuButtonDebouncer.attach(ENCODER_SW_PIN, INPUT_PULLUP);
  menuButtonDebouncer.interval(25);
  menuScreenButtonDebouncer.attach(BTN_MENU, INPUT_PULLUP);
  menuScreenButtonDebouncer.interval(25);

  // Configurar pin del sensor de fin de carrera de la cizalla     
  pinMode(CIZALLA_RETRAIDA_SENSOR_PIN, INPUT_PULLUP); // O INPUT si tiene pull-up/down externo

  // Inicializar display
  tft.begin();
  tft.setRotation(3); // Ajusta la rotación si es necesario
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("Inicializando...");

  // Inicializar encoders
  menuEncoder.attachHalfQuad(ENCODER_CLK_PIN, ENCODER_DT_PIN);
  menuEncoder.clearCount();
  profileEncoder.attachHalfQuad(PROFILE_ENCODER_A, PROFILE_ENCODER_B);
  profileEncoder.clearCount();

  // Inicializar sensores inductivos
  pinMode(SENSOR_IND_1, INPUT);
  pinMode(SENSOR_IND_2, INPUT);

  // Inicializar motores
  setupMotors();

  // Inicializar SD
  initSDCard();

  updateDisplay(); // Dibujar la pantalla inicial
  logMachineState("POWER_ON");
}

void loop() {
  // Actualizar estados de los botones con Bounce2
  startButtonDebouncer.update();
  emergencyButtonDebouncer.update();
  menuButtonDebouncer.update();
  menuScreenButtonDebouncer.update();

  // Manejo de botones
  if (startButtonDebouncer.fell()) {
    handleStartButton();
  }
  if (emergencyButtonDebouncer.fell()) {
    handleEmergencyButton();
  }
  if (menuButtonDebouncer.fell()) {
    handleEncoderButton();
  }
  if (menuScreenButtonDebouncer.fell()) {
    cycleScreens();
  }

  // Simulación de un botón de "MENU" para cambiar de pantalla
  // Puedes usar un pin diferente para esto si quieres un botón dedicado
  // Por ahora, asumiremos que no hay un botón de "MENU" dedicado y se cambia con el encoder switch si no se está editando
  // (Esto podría ser confuso, considera añadir un botón físico si es posible)

  handleMenuEncoder(); // Leer y procesar el encoder del menú

  Serial.print("Encoder perfil: ");
  Serial.println(profileEncoder.getCount());

  // Lógica principal de la máquina basada en el estado
  switch (currentState) {
    case IDLE:
      // Esperar comando de inicio o cambios de configuración
      break;

    case WAITING_FOR_PROFILE:
      if (firstSensorActive()) {
        Serial.println("Primer sensor detecta perfil. Moviendo a posicion de calibracion.");
        currentState = MOVING_TO_OFFSET;
        // Mover el perfil hasta que ambos sensores estén activos
        // En este punto, asumes que el perfil está alineado y listo para ser medido.
        // Asumiendo que el offset de matriz ya está calibrado.
        // MoveProfileBy(distance_to_align_both_sensors); // Si necesitas un movimiento inicial para alineación
        tft.fillScreen(ILI9341_BLACK);
        tft.setCursor(0,0); tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
        tft.println("Alineando perfil...");
      }
      break;

    case MOVING_TO_OFFSET:
      // Mover el perfil lentamente hasta que ambos sensores estén activos
      // Aquí, motor1.moveTo(profileEncoder.getCount() + x);
      // O motor1.runSpeed(); y detener cuando bothSensorsActive()
      if (!bothSensorsActive()) {
          // Mover lentamente hasta que ambos sensores detecten
          motor1.setSpeed(100); motor2.setSpeed(100); motor3.setSpeed(100); // Velocidad lenta
          motor1.runSpeed(); motor2.runSpeed(); motor3.runSpeed();
      } else {
          motor1.stop(); motor2.stop(); motor3.stop();
          while(motor1.isRunning() || motor2.isRunning() || motor3.isRunning()){ // Esperar que se detengan
            motor1.run(); motor2.run(); motor3.run();
          }
          Serial.println("Ambos sensores detectan perfil. Perfil en offset de referencia.");
          profileEncoder.setCount(matrixOffsetSteps); // Reinicia el contador del perfil para que la "distancia 0" sea la matriz
          currentState = READY_TO_CUT;
          Serial.println("Estado: Listo para Cortar.");
          tft.fillScreen(ILI9341_BLACK);
          tft.setCursor(0,0); tft.setTextColor(ILI941_WHITE); tft.setTextSize(2);
          tft.println("Perfil listo!");
          tft.setTextSize(1);
          tft.println("Presione Start para iniciar el ciclo.");
      }
      break;

    case READY_TO_CUT:
      // Esperar a que el usuario presione Start para iniciar el ciclo de cortes
      if (startButtonDebouncer.fell()) { // Si se presiona Start de nuevo
        if (cutsRemaining > 0) {
            currentState = PERFORMING_CUT_CYCLE;
            Serial.println("Iniciando ciclo de corte...");
            logMachineState("CUT_CYCLE_STARTED");
            // Mover para el primer corte
            moveProfileBy(currentCutLength); // Mueve el perfil la longitud deseada
            performCut(); // Realiza el corte
        } else {
            Serial.println("No hay cortes pendientes o la longitud de corte es cero.");
            currentState = IDLE;
            updateDisplay();
        }
      }
      break;

    case PERFORMING_CUT_CYCLE:
      if (cutsRemaining > 0) {
        // Verificar si todavía hay perfil para cortar (el primer sensor debe seguir activo)
        if (firstSensorActive()) {
          // Mover para el siguiente corte
          moveProfileBy(currentCutLength);
          performCut();
        } else {
          Serial.println("Perfil agotado o retirado. Fin del ciclo.");
          handleError("Perfil agotado o retirado."); // Considerar como error o fin de operación
          currentState = IDLE; // O ERROR si lo consideras crítico
          updateDisplay();
        }
      } else {
        Serial.println("Todos los cortes completados. Fin del proceso.");
        currentState = IDLE; // Vuelve al estado inactivo
        updateDisplay();
      }
      break;

    case PAUSED:
      // Lógica de pausa
      break;

    case EMERGENCY_STOP:
      // No hacer nada, esperar que se reinicie manualmente o se resuelva la emergencia
      // Podrías tener un botón específico de "Resetear Emergencia"
      break;

    case ERROR:
      // En este estado, la máquina está detenida hasta que se resuelva el error
      // (Puede requerir reinicio manual o un botón de reset de error)
      break;
  }
}

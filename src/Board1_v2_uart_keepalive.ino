/*
 * ============================================================================
 *  BOARD 1 - PANEL BRACKET SILLON DENTAL   v2 (con control UART)
 *  Firmware LGT8F328P Pro Mini @ 32 MHz / 5V
 * ============================================================================
 *
 *  Dos fuentes de comando independientes sobre las mismas 12 salidas:
 *    (A) 12 switches fisicos con debounce
 *    (B) UART (HC-05 o FTDI) con comandos tipo "SWn TRUE" (keep-alive)
 *
 *  La salida final por cada canal es la union OR de ambas fuentes.
 *  Ninguna bloquea a la otra. UART usa la ISR interna del Serial de Arduino
 *  (ring buffer de 64B) y todo el parsing se hace en el loop.
 *
 *  MAPEO (SW logico -> pin fisico -> salida PCF8574 en Board 2)
 *  ------------------------------------------------------------
 *     SW1  D2   -> PCF @0x20 P0        SW7  D8   -> PCF @0x20 P6
 *     SW2  D3   -> PCF @0x20 P1        SW8  D9   -> PCF @0x20 P7
 *     SW3  D4   -> PCF @0x20 P2        SW9  A0   -> PCF @0x21 P0
 *     SW4  D5   -> PCF @0x20 P3        SW10 A1   -> PCF @0x21 P1
 *     SW5  D6   -> PCF @0x20 P4        SW11 A2   -> PCF @0x21 P2
 *     SW6  D7   -> PCF @0x20 P5        SW12 A3   -> PCF @0x21 P3
 *
 *     A4 (SDA), A5 (SCL)               <-> bus I2C
 *     D0/D1 (UART) 9600 8N1            <-> HC-05 o FTDI (una a la vez)
 *     D13                              LED heartbeat
 *
 *  LOGICA (modo sink de los PC817)
 *  --------------------------------
 *     canal activo   -> bit del PCF en LOW  (0) -> opto ON  -> panel cerrado
 *     canal inactivo -> bit del PCF en HIGH (1) -> opto OFF -> panel abierto
 *
 *  COMANDOS UART (case-insensitive, terminador \n o \r)
 *  ----------------------------------------------------
 *     SW1 TRUE  ... SW12 TRUE   ->  refresca keep-alive del canal (300 ms)
 *     SW1       ... SW12        ->  mismo efecto (TRUE es opcional)
 *     STATUS    (o S)           ->  imprime estado por fuente y combinado
 *     SCAN      (o I)           ->  escanea el bus I2C
 *     ALLOFF                    ->  cancela todos los keep-alive serial
 *     HELP      (o H o ?)       ->  ayuda
 *
 *  KEEP-ALIVE
 *  ----------
 *     Cada "SW_ TRUE" recibido reinicia un temporizador de SERIAL_TIMEOUT_MS
 *     para ese canal. Si no llega otro TRUE antes del timeout, el canal se
 *     libera automaticamente. Recomendado enviar cada 100 ms para tolerar
 *     perdidas puntuales sin desactivar por error.
 * ============================================================================
 */

#include <Wire.h>
#include <avr/wdt.h>
#include <ctype.h>

// ---------------------------------------------------------------------------
// CONFIGURACION
// ---------------------------------------------------------------------------

constexpr uint8_t  PCF_ADDR_LO         = 0x20;   // switches 1..8
constexpr uint8_t  PCF_ADDR_HI         = 0x21;   // switches 9..12
constexpr uint8_t  NUM_SW              = 12;

// Ventana de keep-alive del comando serial
constexpr uint16_t SERIAL_TIMEOUT_MS   = 300;

// Muestreo y debounce de switches fisicos
constexpr uint16_t READ_INTERVAL_MS    = 5;
constexpr uint8_t  DEBOUNCE_SAMPLES    = 3;

// Buffer de linea para UART
constexpr uint8_t  LINE_BUF_SIZE       = 24;
constexpr uint16_t LINE_TIMEOUT_MS     = 500;

// Housekeeping
constexpr uint16_t HEARTBEAT_MS        = 1000;
constexpr uint16_t I2C_RETRY_DELAY_MS  = 20;
constexpr uint8_t  I2C_MAX_RETRIES     = 3;

// Mapeo pin fisico <- indice logico (0..11)
const uint8_t SW_PIN[NUM_SW] = {
   2,  3,  4,  5,  6,  7,  8,  9,     // SW1..SW8  -> D2..D9
  A0, A1, A2, A3                       // SW9..SW12 -> A0..A3
};

// ---------------------------------------------------------------------------
// ESTADO
// ---------------------------------------------------------------------------

// Fuente fisica (con debounce)
bool     phys_state[NUM_SW]      = { false };
uint8_t  phys_debounce[NUM_SW]   = { 0 };

// Fuente serial (keep-alive)
bool     serial_active[NUM_SW]   = { false };
uint32_t serial_last_ms[NUM_SW]  = { 0 };

// Ultimo estado enviado a cada PCF (para no saturar el bus)
uint8_t  last_sent_lo = 0xFF;
uint8_t  last_sent_hi = 0xFF;

// Buffer de linea UART
char     line_buf[LINE_BUF_SIZE];
uint8_t  line_len = 0;
uint32_t line_last_byte_ms = 0;

// Timers periodicos
uint32_t t_read  = 0;
uint32_t t_heart = 0;

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup() {
  wdt_disable();

  Serial.begin(9600);
  delay(50);

  for (uint8_t i = 0; i < NUM_SW; i++) {
    pinMode(SW_PIN[i], INPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();
  Wire.setClock(100000);

  // Estado seguro: todos los canales del panel abiertos (0xFF -> optos OFF)
  writePCF(PCF_ADDR_LO, 0xFF);
  writePCF(PCF_ADDR_HI, 0xFF);
  last_sent_lo = 0xFF;
  last_sent_hi = 0xFF;

  wdt_enable(WDTO_2S);

  Serial.println();
  Serial.println(F("============================================"));
  Serial.println(F("  BOARD 1 v2 - Panel bracket sillon dental"));
  Serial.println(F("  LGT8F328P | 12 SW fisicos + UART keep-alive"));
  Serial.println(F("============================================"));
  Serial.print (F("  Timeout keep-alive: "));
  Serial.print (SERIAL_TIMEOUT_MS);
  Serial.println(F(" ms"));
  Serial.println();

  scanI2C();

  Serial.println(F("Listo. Comandos: 'HELP' para ayuda."));
  Serial.println();
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------

void loop() {
  wdt_reset();
  uint32_t now = millis();

  // 1) Muestreo de switches fisicos (con debounce)
  if (now - t_read >= READ_INTERVAL_MS) {
    t_read = now;
    scanPhysicalSwitches();
  }

  // 2) UART: siempre poll (no bloqueante)
  pollSerial(now);

  // 3) Expiracion de keep-alives seriales
  expireSerialTimeouts(now);

  // 4) Recalcular estado combinado y transmitir si cambio
  computeAndSendState();

  // 5) Heartbeat visual
  if (now - t_heart >= HEARTBEAT_MS) {
    t_heart = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

// ---------------------------------------------------------------------------
// FUENTE A: SWITCHES FISICOS CON DEBOUNCE
// ---------------------------------------------------------------------------

void scanPhysicalSwitches() {
  for (uint8_t i = 0; i < NUM_SW; i++) {
    bool reading = (digitalRead(SW_PIN[i]) == HIGH);

    if (reading == phys_state[i]) {
      phys_debounce[i] = 0;
    } else {
      phys_debounce[i]++;
      if (phys_debounce[i] >= DEBOUNCE_SAMPLES) {
        phys_state[i] = reading;
        phys_debounce[i] = 0;
        Serial.print(F("[EVT-PHYS] SW"));
        Serial.print(i + 1);
        Serial.println(reading ? F(" PRESIONADO") : F(" libre"));
      }
    }
  }
}

// ---------------------------------------------------------------------------
// FUENTE B: UART - POLLING NO BLOQUEANTE
// ---------------------------------------------------------------------------

void pollSerial(uint32_t now) {
  // Timeout de linea parcial: descartar si empezo pero se corto
  if (line_len > 0 && (now - line_last_byte_ms) > LINE_TIMEOUT_MS) {
    line_len = 0;
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    line_last_byte_ms = now;

    if (c == '\n' || c == '\r') {
      if (line_len > 0) {
        line_buf[line_len] = '\0';
        parseAndDispatch(line_buf);
        line_len = 0;
      }
    } else if (line_len < LINE_BUF_SIZE - 1) {
      line_buf[line_len++] = c;
    } else {
      // Overflow: descartar linea corrupta
      Serial.println(F("[ERR] line overflow"));
      line_len = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// PARSER: separa comando y argumento
// ---------------------------------------------------------------------------

void parseAndDispatch(char* line) {
  // Trim + uppercase in-place
  char* p = line;
  while (*p) { *p = toupper((unsigned char)*p); p++; }

  // Primer token
  char* cmd = strtok(line, " \t");
  if (!cmd) return;

  // Segundo token (opcional)
  char* arg = strtok(NULL, " \t");

  // --- Comando SWn ---
  if (cmd[0] == 'S' && cmd[1] == 'W' && isdigit((unsigned char)cmd[2])) {
    int n = atoi(&cmd[2]);
    if (n >= 1 && n <= NUM_SW) {
      // Aceptar "SWn", "SWn TRUE", "SWn 1"
      bool ok = (arg == NULL)
             || (strcmp(arg, "TRUE") == 0)
             || (strcmp(arg, "1") == 0)
             || (strcmp(arg, "ON") == 0);
      if (ok) {
        refreshSerialKeepAlive(n - 1);
      } else if (strcmp(arg, "FALSE") == 0
              || strcmp(arg, "0") == 0
              || strcmp(arg, "OFF") == 0) {
        // Liberar manualmente ese canal
        serial_active[n - 1] = false;
        Serial.print(F("[EVT-SRL] SW")); Serial.print(n);
        Serial.println(F(" liberado (manual)"));
      } else {
        Serial.print(F("[ERR] arg invalido para SW"));
        Serial.println(n);
      }
      return;
    }
    Serial.println(F("[ERR] SW fuera de rango (1..12)"));
    return;
  }

  // --- Otros comandos ---
  if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "S") == 0) {
    printStatus();
    return;
  }
  if (strcmp(cmd, "SCAN") == 0 || strcmp(cmd, "I") == 0) {
    scanI2C();
    return;
  }
  if (strcmp(cmd, "ALLOFF") == 0) {
    for (uint8_t i = 0; i < NUM_SW; i++) serial_active[i] = false;
    Serial.println(F("[OK] todos los keep-alive serial cancelados"));
    return;
  }
  if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "H") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
    return;
  }

  Serial.print(F("[ERR] comando desconocido: "));
  Serial.println(cmd);
}

// Refresca el temporizador de keep-alive de un canal
void refreshSerialKeepAlive(uint8_t idx) {
  bool was_active = serial_active[idx];
  serial_active[idx]  = true;
  serial_last_ms[idx] = millis();
  if (!was_active) {
    Serial.print(F("[EVT-SRL] SW"));
    Serial.print(idx + 1);
    Serial.println(F(" activo (keep-alive iniciado)"));
  }
}

// Libera canales cuyo keep-alive expiro
void expireSerialTimeouts(uint32_t now) {
  for (uint8_t i = 0; i < NUM_SW; i++) {
    if (serial_active[i] && (now - serial_last_ms[i]) >= SERIAL_TIMEOUT_MS) {
      serial_active[i] = false;
      Serial.print(F("[EVT-SRL] SW"));
      Serial.print(i + 1);
      Serial.println(F(" liberado (timeout)"));
    }
  }
}

// ---------------------------------------------------------------------------
// COMBINAR FUENTES Y ENVIAR POR I2C
// ---------------------------------------------------------------------------

void computeAndSendState() {
  // Estado final: OR de fisico y serial, por canal
  uint8_t bits_lo = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (phys_state[i] || serial_active[i]) bits_lo |= (1 << i);
  }
  uint8_t bits_hi = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (phys_state[8 + i] || serial_active[8 + i]) bits_hi |= (1 << i);
  }

  // Invertir para modo sink (activo -> LOW en PCF)
  // Bits 4..7 de HI estan en 0 (canales sin usar); tras invertir quedan en 1
  // -> salidas HIGH -> optos OFF. Correcto.
  uint8_t pcf_lo = ~bits_lo;
  uint8_t pcf_hi = ~bits_hi;

  if (pcf_lo != last_sent_lo) {
    if (writePCF(PCF_ADDR_LO, pcf_lo)) last_sent_lo = pcf_lo;
  }
  if (pcf_hi != last_sent_hi) {
    if (writePCF(PCF_ADDR_HI, pcf_hi)) last_sent_hi = pcf_hi;
  }
}

// ---------------------------------------------------------------------------
// I2C
// ---------------------------------------------------------------------------

bool writePCF(uint8_t addr, uint8_t data) {
  for (uint8_t attempt = 0; attempt < I2C_MAX_RETRIES; attempt++) {
    Wire.beginTransmission(addr);
    Wire.write(data);
    uint8_t err = Wire.endTransmission();
    if (err == 0) return true;

    Serial.print(F("[I2C ERR] @0x")); Serial.print(addr, HEX);
    Serial.print(F(" err=")); Serial.print(err);
    Serial.print(F(" retry=")); Serial.println(attempt + 1);
    delay(I2C_RETRY_DELAY_MS);
    wdt_reset();
  }
  Serial.print(F("[I2C FAIL] @0x")); Serial.println(addr, HEX);
  return false;
}

void scanI2C() {
  Serial.println(F("Escaneando bus I2C..."));
  bool haveLo = false, haveHi = false;
  uint8_t found = 0;

  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  [OK] 0x")); Serial.println(addr, HEX);
      if (addr == PCF_ADDR_LO) haveLo = true;
      if (addr == PCF_ADDR_HI) haveHi = true;
      found++;
    }
    wdt_reset();
  }
  Serial.print(F("Dispositivos detectados: ")); Serial.println(found);
  if (!haveLo) {
    Serial.print(F("  ADVERTENCIA: PCF @0x"));
    Serial.print(PCF_ADDR_LO, HEX); Serial.println(F(" NO responde"));
  }
  if (!haveHi) {
    Serial.print(F("  ADVERTENCIA: PCF @0x"));
    Serial.print(PCF_ADDR_HI, HEX); Serial.println(F(" NO responde"));
  }
  if (haveLo && haveHi) Serial.println(F("  Ambos PCF OK."));
  Serial.println();
}

// ---------------------------------------------------------------------------
// DEBUG
// ---------------------------------------------------------------------------

void printStatus() {
  Serial.println(F("--- Estado por canal ---"));
  Serial.println(F("  #    fisico  serial  final"));
  for (uint8_t i = 0; i < NUM_SW; i++) {
    bool f = phys_state[i];
    bool s = serial_active[i];
    Serial.print(F("  SW"));
    if (i + 1 < 10) Serial.print(' ');
    Serial.print(i + 1);
    Serial.print(F("  "));
    Serial.print(f ? F(" ON ") : F(" -- "));
    Serial.print(F("   "));
    Serial.print(s ? F(" ON ") : F(" -- "));
    Serial.print(F("   "));
    Serial.println((f || s) ? F("ACTIVO") : F("libre"));
  }
  Serial.print(F("  PCF LO ultimo envio: 0x")); Serial.println(last_sent_lo, HEX);
  Serial.print(F("  PCF HI ultimo envio: 0x")); Serial.println(last_sent_hi, HEX);
  Serial.println();
}

void printHelp() {
  Serial.println(F("--- Comandos ---"));
  Serial.println(F("  SWn        refresca keep-alive de SWn (n=1..12)"));
  Serial.println(F("  SWn TRUE   idem (TRUE / 1 / ON aceptados)"));
  Serial.println(F("  SWn FALSE  libera manualmente ese canal"));
  Serial.println(F("  STATUS     estado por canal y fuente"));
  Serial.println(F("  SCAN       escanea el bus I2C"));
  Serial.println(F("  ALLOFF     cancela todos los keep-alive serial"));
  Serial.println(F("  HELP       esta ayuda"));
  Serial.print  (F("Timeout keep-alive: "));
  Serial.print  (SERIAL_TIMEOUT_MS);
  Serial.println(F(" ms. Enviar cada 100 ms para mantener activo."));
  Serial.println();
}

/*
  M.I.C.H.I. - Collar localizador de gatos (v2: geocerca por SMS)
  ESP32-C3 Super Mini + Quectel BG95-M3 (SMS) + u-blox NEO-6M (GPS)
  ---------------------------------------------------------------------
  COMO FUNCIONA
  1) El GPS va actualizando la posicion por su cuenta (en modo ahorro, si esta activo).
  2) Mandas "SET" por SMS -> el collar fija su posicion actual como CENTRO (la casa).
  3) Cada 10 s compara la posicion contra el centro. Si el gato sale del radio
     (por defecto 50 m) durante 3 lecturas seguidas, manda un SMS de ALERTA con el mapa.
  4) Mientras siga afuera, repite la ubicacion por SMS cada X minutos (por defecto 15).
  5) Cuando vuelve a la zona segura, avisa y deja de mandar mensajes.

  COMANDOS POR SMS (solo desde NUMERO_AUTORIZADO, el collar contesta por SMS)
    SET            Fija la posicion GPS actual como centro
    UBI            Responde con un link de Google Maps de la posicion actual
    RADIO=100      Cambia el radio de la zona segura (1 a 1000 m)
    INTERVALO=15   Cada cuantos minutos avisa mientras esta afuera (1 a 1440)
    PSM_ON         GPS en modo bajo consumo
    PSM_OFF        GPS en modo normal (actualizaciones frecuentes)
    LOG            Info del primer fix logrado desde el ultimo encendido registrado
    LOG_CLEAR      Borra ese registro
    ESTADO         Resumen: centro, radio, intervalo, modo GPS, dentro/fuera

  CONEXIONES
  - BG95-M3 : TX del modulo -> GPIO20 | RX del modulo -> GPIO21   (UART1)
  - NEO-6M  : TX del GPS    -> GPIO3  | RX del GPS    -> GPIO4    (UART0 remapeado)
  - BG95-M3 : pin de encendido (PWRKEY) -> GPIO5   (el ESP32 lo prende solo al arrancar)
  - GND compartido. Datos siempre a 3.3V logicos.

  IDE de Arduino: Tools -> "USB CDC On Boot: Enabled" (para que UART0 quede libre para el GPS).
  Libreria necesaria: TinyGPSPlus (Mikal Hart)
*/

#include <TinyGPSPlus.h>
#include <EEPROM.h>

// ======================= CONFIGURACION =======================

// Unico numero que puede mandar comandos (y al que se le responde / avisa).
// Con codigo de pais. Se comparan los ultimos 8 digitos, asi que da igual +54 / +549.
const char* NUMERO_AUTORIZADO = "+5493517361327";   // <-- CAMBIAR

// Pines
const int RX_MODEM = 20;   // ESP32 recibe  <- TX del BG95
const int TX_MODEM = 21;   // ESP32 envia   -> RX del BG95
const int RX_GPS   = 3;    // ESP32 recibe  <- TX del NEO-6M
const int TX_GPS   = 4;    // ESP32 envia   -> RX del NEO-6M

// Encendido del BG95-M3 (pin de power del modulo, conectado al GPIO5)
const int           PIN_PWRKEY         = 5;
const bool          PWRKEY_ACTIVO_ALTO = true;   // true = se prende con un pulso ALTO (3.3 V). Si no prende, probar false
const unsigned long PWRKEY_PULSO_MS    = 1000;   // Duracion del pulso (1 s)
const unsigned long ARRANQUE_MODULO_MS = 6000;   // Espera despues del pulso hasta que el modulo arranca

// Valores por defecto (se pueden cambiar por SMS y quedan guardados en flash)
const float    RADIO_DEFECTO     = 50.0;   // metros
const uint16_t INTERVALO_DEFECTO = 15;     // minutos entre avisos mientras esta afuera
const byte     PSM_DEFECTO       = 1;      // 1 = GPS en ahorro de energia

// Tiempos y filtros
const unsigned long TIMEOUT_FIX_MS        = 90000;   // Espera maxima de un fix por cada pedido (SET / UBI)
const unsigned long INTERVALO_CHEQUEO_MS  = 10000;   // Cada cuanto se compara la posicion con el centro
const unsigned long MAX_EDAD_FIX_MS       = 60000;   // Un fix mas viejo que esto no se usa para decidir dentro/fuera
const int           LECTURAS_CONFIRMAR    = 3;       // Lecturas seguidas para confirmar que salio/entro (filtra el "salto" del GPS)
const unsigned long INTERVALO_REVISION_MS = 15000;   // Cada cuanto se revisan SMS nuevos (respaldo del aviso +CMTI)
const int           FALLOS_PARA_REINICIAR = 5;       // Fallos seguidos del modem antes de reiniciarlo

// Comandos UBX (solo chips u-blox como el NEO-6M)
const byte UBX_PSM_ON[]  = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
const byte UBX_PSM_OFF[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};

// ---------- Memoria flash (EEPROM emulada) ----------
const int  EEPROM_SIZE     = 128;
const int  EEPROM_DIR_CFG  = 0;      // Configuracion y centro
const int  EEPROM_DIR_LOG  = 64;     // Registro del primer fix
const byte EEPROM_CFG_FLAG = 0xB1;   // Sello: la configuracion es valida
const byte EEPROM_LOG_FLAG = 0x5A;   // Sello: el registro es valido

struct Config {
  byte     flag;
  byte     psm;           // 1 = ahorro de energia del GPS activo
  byte     centroFijado;  // 1 = ya hay centro guardado
  uint16_t intervaloMin;  // Minutos entre avisos mientras esta afuera
  float    radio;         // Metros
  double   lat;           // Centro (casa)
  double   lon;
};

struct RegistroPrueba {
  byte          flag;
  unsigned long tiempoMs;   // Tiempo desde el encendido hasta el primer fix
  byte          satelites;
  double        lat;
  double        lon;
};

// ======================= OBJETOS Y VARIABLES =======================

HardwareSerial modem(1);       // UART1 -> BG95-M3
HardwareSerial gpsSerial(0);   // UART0 (remapeado a GPIO3/4) -> NEO-6M
TinyGPSPlus gps;

Config cfg;

String linea = "";             // Linea que arma enviarAT()
String lineaURC = "";          // Linea que arma el loop para detectar avisos espontaneos del modem
bool revisarYa = false;        // true = el modem aviso que llego un SMS (+CMTI)
unsigned long ultimaRevision = 0;
int fallosSeguidos = 0;

// Estado de la geocerca
bool afuera = false;
int lecturasFuera = 0;
int lecturasDentro = 0;
unsigned long ultimoAvisoFuera = 0;
unsigned long ultimoChequeo = 0;

bool fixYaRegistrado = false;  // true = en este encendido ya se guardo el primer fix

// ======================= CONFIGURACION EN FLASH =======================

void guardarConfig() {
  EEPROM.put(EEPROM_DIR_CFG, cfg);
  EEPROM.commit();   // Sin esto, los datos no quedan grabados de verdad
}

void cargarConfig() {
  EEPROM.get(EEPROM_DIR_CFG, cfg);

  if (cfg.flag != EEPROM_CFG_FLAG) {
    // Primera vez: valores de fabrica
    cfg.flag = EEPROM_CFG_FLAG;
    cfg.psm = PSM_DEFECTO;
    cfg.centroFijado = 0;
    cfg.intervaloMin = INTERVALO_DEFECTO;
    cfg.radio = RADIO_DEFECTO;
    cfg.lat = 0.0;
    cfg.lon = 0.0;
    guardarConfig();
  }
}

// ======================= GPS =======================

// Pasa al TinyGPS todo lo que haya llegado del GPS. Se llama seguido,
// incluso mientras se espera al modem, para no perder datos.
void alimentarGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

void gpsModoContinuo() {
  gpsSerial.write(UBX_PSM_OFF, sizeof(UBX_PSM_OFF));
}

void gpsModoAhorro() {
  gpsSerial.write(UBX_PSM_ON, sizeof(UBX_PSM_ON));
}

// Deja al GPS en el modo que corresponde segun la configuracion guardada
void gpsAplicarModoConfigurado() {
  if (cfg.psm) gpsModoAhorro();
  else gpsModoContinuo();
}

bool fixReciente() {
  return gps.location.isValid() && gps.location.age() < MAX_EDAD_FIX_MS;
}

// Espera hasta tener un fix valido y fresco. Devuelve false si se acaba el tiempo.
bool esperarFixGPS(unsigned long timeout) {
  Serial.println("\nBuscando coordenadas GPS...");
  unsigned long inicio = millis();
  unsigned long ultimoAviso = 0;

  while (millis() - inicio < timeout) {
    alimentarGPS();

    if (gps.location.isValid() && gps.location.age() < 2000) {
      Serial.println("Fix GPS obtenido.");
      return true;
    }

    if (millis() - ultimoAviso >= 5000) {
      ultimoAviso = millis();
      Serial.print("  ... esperando fix | satelites: ");
      Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
      Serial.print(" | caracteres recibidos: ");
      Serial.println(gps.charsProcessed());
    }
  }

  Serial.println("TIMEOUT: no hubo fix GPS.");
  return false;
}

String linkMapa() {
  return "https://www.google.com/maps?q=" +
         String(gps.location.lat(), 6) + "," +
         String(gps.location.lng(), 6);
}

// Formula de Haversine: distancia en linea recta entre dos coordenadas, en metros
double calcularDistanciaMetros(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// ======================= REGISTRO DEL PRIMER FIX =======================

void registrarPrimerFix() {
  fixYaRegistrado = true;

  RegistroPrueba r;
  r.flag = EEPROM_LOG_FLAG;
  r.tiempoMs = millis();
  r.satelites = gps.satellites.value();
  r.lat = gps.location.lat();
  r.lon = gps.location.lng();

  EEPROM.put(EEPROM_DIR_LOG, r);
  EEPROM.commit();

  Serial.println(">>> Primer fix logrado y guardado en la memoria. <<<");
}

// ======================= MODEM (AT) =======================

// Envia un comando AT y espera OK / ERROR.
// Si se pasa 'resp', se guarda ahi todo lo que contesto el modem.
bool enviarAT(String comando, unsigned long timeout = 20000, String* resp = nullptr) {

  while (modem.available()) {
    modem.read();
  }

  linea = "";
  if (resp) *resp = "";

  Serial.println();
  Serial.print("TX: ");
  Serial.println(comando);

  // Enviar comando terminado solamente en CR
  modem.print(comando);
  modem.write('\r');

  unsigned long inicio = millis();

  while (millis() - inicio < timeout) {

    alimentarGPS();

    while (modem.available()) {

      char c = modem.read();
      Serial.write(c);
      if (resp) *resp += c;

      if (c == '\n') {
        linea.trim();

        if (linea == "OK") {
          Serial.println("\nComando correcto.");
          return true;
        }

        if (linea.startsWith("ERROR") ||
            linea.startsWith("+CME ERROR") ||
            linea.startsWith("+CMS ERROR")) {
          Serial.println("\nEl modulo devolvio un error.");
          return false;
        }

        linea = "";
      }
      else if (c != '\r') {
        linea += c;
      }
    }
  }

  Serial.println("\nTIMEOUT: el modulo no respondio a tiempo.");
  return false;
}

// Espera a que el modem mande cierto texto (por ejemplo el prompt ">" del SMS).
bool esperarTexto(const char* objetivo, unsigned long timeout) {
  String acum = "";
  unsigned long inicio = millis();

  while (millis() - inicio < timeout) {
    alimentarGPS();

    while (modem.available()) {
      char c = modem.read();
      Serial.write(c);
      acum += c;

      if (acum.indexOf(objetivo) >= 0) return true;
      if (acum.indexOf("ERROR") >= 0) return false;
    }
  }
  return false;
}

// Deja el pin PWRKEY "en reposo" (sin pulso)
void pwrkeyReposo() {
  if (PWRKEY_ACTIVO_ALTO) {
    pinMode(PIN_PWRKEY, OUTPUT);
    digitalWrite(PIN_PWRKEY, LOW);
  } else {
    pinMode(PIN_PWRKEY, INPUT);   // Alta impedancia: no fuerza ningun nivel
  }
}

// Manda el nivel "activo" al pin PWRKEY
void pwrkeyActivo() {
  pinMode(PIN_PWRKEY, OUTPUT);
  digitalWrite(PIN_PWRKEY, PWRKEY_ACTIVO_ALTO ? HIGH : LOW);
}

// Prende el BG95-M3 con un pulso en su pin PWRKEY (GPIO5).
// Si el modulo YA esta prendido (por ejemplo, el ESP32 se reinicio pero el modulo siguio con
// energia), NO se manda el pulso: el mismo pulso lo APAGARIA.
void encenderModulo() {

  Serial.println("\n===== ENCENDIENDO BG95-M3 =====");

  // Primero verificar si ya esta prendido
  for (int i = 0; i < 2; i++) {
    if (enviarAT("AT", 1500)) {
      Serial.println("El modulo ya estaba prendido. No hace falta el pulso.");
      return;
    }
  }

  Serial.println("El modulo no responde: mandando pulso de encendido...");
  pwrkeyActivo();
  delay(PWRKEY_PULSO_MS);
  pwrkeyReposo();

  Serial.println("Esperando a que arranque el modulo...");
  delay(ARRANQUE_MODULO_MS);
}

// Esperar hasta que el modulo vuelva a responder
bool esperarModulo() {

  Serial.println("\nEsperando al BG95-M3...");

  for (int i = 0; i < 20; i++) {

    if (enviarAT("AT", 3000)) {
      Serial.println("BG95-M3 disponible.");
      return true;
    }

    delay(2000);
  }

  return false;
}

// Reiniciar el modulo
void reiniciarModulo() {

  Serial.println("\n===== REINICIANDO MODULO =====");

  enviarAT("AT+CFUN=1,1", 10000);

  // Dar tiempo para reiniciarse
  delay(45000);

  esperarModulo();
}

// Lee el estado de registro de red (CEREG / CREG). Devuelve el codigo <stat> o -1.
int estadoRegistro(String comando, String etiqueta) {
  String r;
  if (!enviarAT(comando, 10000, &r)) return -1;

  int i = r.indexOf(etiqueta);
  if (i < 0) return -1;
  int coma = r.indexOf(',', i);
  if (coma < 0) return -1;
  return r.substring(coma + 1, coma + 2).toInt();
}

// Espera a que el modem se registre en la red (LTE o GSM). 1 = local, 5 = roaming.
bool esperarRed(unsigned long timeout) {
  Serial.println("\nEsperando registro en la red...");
  unsigned long inicio = millis();

  while (millis() - inicio < timeout) {
    int lte = estadoRegistro("AT+CEREG?", "+CEREG:");
    int gsm = estadoRegistro("AT+CREG?", "+CREG:");

    if (lte == 1 || lte == 5 || gsm == 1 || gsm == 5) {
      Serial.println("Registrado en la red.");
      return true;
    }
    delay(3000);
  }

  Serial.println("No se logro registrar en la red a tiempo.");
  return false;
}

// Secuencia de inicio
bool iniciarModulo() {

  Serial.println("\n===== CONFIGURANDO BG95-M3 =====");

  if (!enviarAT("AT")) return false;

  // Desactivar eco y activar respuestas detalladas
  if (!enviarAT("ATE0;V1")) return false;

  // Seleccionar perfil comercial
  if (!enviarAT("AT+QMBNCFG=\"autosel\",0")) return false;

  if (!enviarAT("AT+QMBNCFG=\"select\",\"ROW_Commercial\"", 30000))
    return false;

  // GSM + LTE automatico
  if (!enviarAT("AT+QCFG=\"nwscanmode\",0,1", 20000))
    return false;

  // Secuencia de busqueda
  if (!enviarAT("AT+QCFG=\"nwscanseq\",02,1", 20000))
    return false;

  // Dominio de servicio: CS + PS
  if (!enviarAT("AT+QCFG=\"servicedomain\",2,1", 20000))
    return false;

  // Modo LTE Cat-M1
  if (!enviarAT("AT+QCFG=\"iotopmode\",0,1", 20000))
    return false;

  // ----- Configuracion de SMS -----
  // Modo texto (obligatorio para trabajar con SMS en texto plano)
  if (!enviarAT("AT+CMGF=1")) return false;

  // Juego de caracteres GSM
  enviarAT("AT+CSCS=\"GSM\"");

  // Avisar con +CMTI cuando llegue un SMS nuevo (queda guardado en memoria)
  enviarAT("AT+CNMI=2,1,0,0,0");

  // Limpiar mensajes viejos que hayan quedado guardados
  enviarAT("AT+CMGD=1,4");

  Serial.println("\n===== CONFIGURACION TERMINADA =====");

  return true;
}

// ======================= SMS: ENVIO =======================

// Manda un SMS de texto. Devuelve true si el modem confirmo el envio.
bool enviarSMS(String numero, String texto) {

  while (modem.available()) {
    modem.read();
  }

  Serial.println();
  Serial.print("Enviando SMS a ");
  Serial.println(numero);

  modem.print("AT+CMGS=\"");
  modem.print(numero);
  modem.print("\"\r");

  // El modem contesta con el prompt ">" cuando esta listo para recibir el texto
  if (!esperarTexto(">", 10000)) {
    Serial.println("\nNo aparecio el prompt '>'. Cancelando.");
    modem.write(0x1B);   // ESC: cancela el envio
    return false;
  }

  modem.print(texto);
  modem.write(0x1A);     // Ctrl+Z: fin del mensaje, lo manda

  // Esperar la confirmacion final (+CMGS: n ... OK)
  if (esperarTexto("OK", 60000)) {
    Serial.println("\nSMS enviado.");
    return true;
  }

  Serial.println("\nFallo el envio del SMS.");
  return false;
}

// Manda un SMS al numero autorizado, con hasta 3 intentos
bool responder(String texto) {
  for (int intento = 1; intento <= 3; intento++) {
    if (enviarSMS(String(NUMERO_AUTORIZADO), texto)) return true;
    Serial.print("Reintentando envio (");
    Serial.print(intento);
    Serial.println("/3)...");
    delay(3000);
  }
  return false;
}

// ======================= GEOCERCA =======================

// Se llama cada INTERVALO_CHEQUEO_MS. Compara la posicion actual con el centro.
void revisarPerimetro() {
  if (!cfg.centroFijado) return;

  unsigned long intervaloMs = (unsigned long)cfg.intervaloMin * 60000UL;

  // Sin posicion reciente no podemos decidir dentro/fuera. Pero si ya estaba afuera
  // y toca avisar, mandamos la ultima posicion conocida.
  if (!fixReciente()) {
    if (afuera && millis() - ultimoAvisoFuera >= intervaloMs) {
      ultimoAvisoFuera = millis();
      if (gps.location.isValid()) {
        responder("M.I.C.H.I. sigue FUERA de la zona segura pero sin senal GPS ahora. Ultima posicion conocida:\n" + linkMapa());
      } else {
        responder("M.I.C.H.I. sigue FUERA de la zona segura y no hay senal GPS.");
      }
    }
    return;
  }

  double d = calcularDistanciaMetros(cfg.lat, cfg.lon, gps.location.lat(), gps.location.lng());

  Serial.print("\n[Geocerca] Distancia al centro: ");
  Serial.print(d, 1);
  Serial.print(" m (radio ");
  Serial.print(cfg.radio, 0);
  Serial.println(" m)");

  if (d > cfg.radio) {
    lecturasFuera++;
    lecturasDentro = 0;
  } else {
    lecturasDentro++;
    lecturasFuera = 0;
  }

  if (!afuera && lecturasFuera >= LECTURAS_CONFIRMAR) {
    // Salio de la zona segura: alerta inmediata
    afuera = true;
    ultimoAvisoFuera = millis();
    responder("ALERTA M.I.C.H.I.: salio de la zona segura (" + String((int)d) + " m del centro).\n" + linkMapa());
  }
  else if (afuera && lecturasDentro >= LECTURAS_CONFIRMAR) {
    // Volvio a la zona segura
    afuera = false;
    responder("M.I.C.H.I. volvio a la zona segura.");
  }
  else if (afuera && millis() - ultimoAvisoFuera >= intervaloMs) {
    // Sigue afuera: aviso periodico
    ultimoAvisoFuera = millis();
    responder("M.I.C.H.I. sigue fuera (" + String((int)d) + " m del centro).\n" + linkMapa());
  }
}

// ======================= COMANDOS =======================

void comandoSet() {
  if (cfg.psm) gpsModoContinuo();   // Para conseguir el fix lo mas rapido posible
  bool ok = esperarFixGPS(TIMEOUT_FIX_MS);
  gpsAplicarModoConfigurado();

  if (!ok) {
    responder("M.I.C.H.I. no pudo fijar el centro: sin senal GPS. Proba al aire libre y reintenta.");
    return;
  }

  cfg.lat = gps.location.lat();
  cfg.lon = gps.location.lng();
  cfg.centroFijado = 1;
  guardarConfig();

  afuera = false;
  lecturasFuera = 0;
  lecturasDentro = 0;

  responder("M.I.C.H.I. centro fijado. Radio: " + String(cfg.radio, 0) + " m.\n" + linkMapa());
}

void comandoUbicacion() {
  if (cfg.psm) gpsModoContinuo();
  bool hayFix = esperarFixGPS(TIMEOUT_FIX_MS);
  gpsAplicarModoConfigurado();

  String mensaje;

  if (hayFix) {
    mensaje = "M.I.C.H.I. ubicacion actual:\n" + linkMapa() +
              "\nSatelites: " + String(gps.satellites.isValid() ? gps.satellites.value() : 0);
  }
  else if (gps.location.isValid()) {
    unsigned long minutos = gps.location.age() / 60000UL;
    mensaje = "M.I.C.H.I. sin senal GPS ahora. Ultima posicion conocida (hace " +
              String(minutos) + " min):\n" + linkMapa();
  }
  else {
    mensaje = "M.I.C.H.I. no pudo obtener posicion GPS (sin senal o sin cielo abierto). Reintenta en unos minutos.";
  }

  responder(mensaje);
}

void comandoLog() {
  RegistroPrueba r;
  EEPROM.get(EEPROM_DIR_LOG, r);

  if (r.flag != EEPROM_LOG_FLAG) {
    responder("M.I.C.H.I. LOG vacio: todavia no hay ningun fix registrado.");
    return;
  }

  responder("M.I.C.H.I. primer fix registrado: tardo " + String(r.tiempoMs / 1000.0, 1) +
            " s desde el encendido | Sat: " + String(r.satelites) +
            "\nhttps://www.google.com/maps?q=" + String(r.lat, 6) + "," + String(r.lon, 6));
}

void comandoLogClear() {
  RegistroPrueba r;
  r.flag = 0x00;
  r.tiempoMs = 0;
  r.satelites = 0;
  r.lat = 0;
  r.lon = 0;

  EEPROM.put(EEPROM_DIR_LOG, r);
  EEPROM.commit();

  responder("M.I.C.H.I. registro borrado.");
}

void comandoEstado() {
  String m = "M.I.C.H.I. estado:\n";
  m += "Centro: " + String(cfg.centroFijado ? "fijado" : "SIN FIJAR") + "\n";
  m += "Radio: " + String(cfg.radio, 0) + " m\n";
  m += "Aviso c/" + String(cfg.intervaloMin) + " min\n";
  m += "GPS: " + String(cfg.psm ? "ahorro" : "normal") + "\n";
  m += "Zona: " + String(!cfg.centroFijado ? "-" : (afuera ? "FUERA" : "dentro"));
  responder(m);
}

// Interpreta un comando de SMS (ya en mayusculas y sin espacios en los extremos)
void procesarComando(String c) {

  Serial.print("\n>>> Comando: ");
  Serial.println(c);

  if (c == "SET") {
    comandoSet();
  }
  else if (c == "UBI") {
    comandoUbicacion();
  }
  else if (c == "PSM_ON") {
    cfg.psm = 1;
    guardarConfig();
    gpsModoAhorro();
    responder("M.I.C.H.I. GPS en modo ahorro (PSM). Las posiciones se actualizan mas lento.");
  }
  else if (c == "PSM_OFF") {
    cfg.psm = 0;
    guardarConfig();
    gpsModoContinuo();
    responder("M.I.C.H.I. GPS en modo normal (actualizaciones frecuentes).");
  }
  else if (c == "LOG") {
    comandoLog();
  }
  else if (c == "LOG_CLEAR") {
    comandoLogClear();
  }
  else if (c == "ESTADO") {
    comandoEstado();
  }
  else if (c.startsWith("RADIO=")) {
    int v = c.substring(6).toInt();
    if (v < 1 || v > 1000) {
      responder("M.I.C.H.I. RADIO invalido. Usa un valor entre 1 y 1000 (metros). Ej: RADIO=100");
    } else {
      cfg.radio = v;
      guardarConfig();
      lecturasFuera = 0;
      lecturasDentro = 0;
      responder("M.I.C.H.I. radio de la zona segura: " + String(v) + " m.");
    }
  }
  else if (c.startsWith("INTERVALO=")) {
    int v = c.substring(10).toInt();
    if (v < 1 || v > 1440) {
      responder("M.I.C.H.I. INTERVALO invalido. Usa minutos entre 1 y 1440. Ej: INTERVALO=15");
    } else {
      cfg.intervaloMin = v;
      guardarConfig();
      responder("M.I.C.H.I. aviso cada " + String(v) + " min mientras este fuera.");
    }
  }
  else {
    responder("M.I.C.H.I. comando desconocido. Disponibles: SET, UBI, RADIO=n, INTERVALO=min, PSM_ON, PSM_OFF, LOG, LOG_CLEAR, ESTADO");
  }
}

// ======================= SMS: RECEPCION =======================

// Se queda con los ultimos 8 digitos de un numero, para comparar sin importar el formato (+54, +549, 0, 15...)
String ultimosDigitos(String numero) {
  String d = "";
  for (unsigned int i = 0; i < numero.length(); i++) {
    if (isDigit(numero[i])) d += numero[i];
  }
  if (d.length() > 8) d = d.substring(d.length() - 8);
  return d;
}

bool esNumeroAutorizado(String remitente) {
  return ultimosDigitos(remitente) == ultimosDigitos(String(NUMERO_AUTORIZADO));
}

// Lee los SMS sin leer y ejecuta los comandos que vengan del numero autorizado.
void revisarSMS() {

  String resp;

  if (!enviarAT("AT+CMGL=\"REC UNREAD\"", 10000, &resp)) {
    fallosSeguidos++;
    return;
  }
  fallosSeguidos = 0;

  const int MAX_COMANDOS = 5;
  String comandos[MAX_COMANDOS];
  int nComandos = 0;
  int pos = 0;

  while (true) {
    int ini = resp.indexOf("+CMGL:", pos);
    if (ini < 0) break;

    int finCabecera = resp.indexOf('\n', ini);
    if (finCabecera < 0) break;

    // Cabecera: +CMGL: <indice>,"REC UNREAD","<remitente>","","<fecha>"
    String cabecera = resp.substring(ini, finCabecera);
    int q1 = cabecera.indexOf('"');
    int q2 = cabecera.indexOf('"', q1 + 1);
    int q3 = cabecera.indexOf('"', q2 + 1);
    int q4 = cabecera.indexOf('"', q3 + 1);
    String remitente = (q3 >= 0 && q4 > q3) ? cabecera.substring(q3 + 1, q4) : "";

    // El texto del SMS viene en la linea siguiente
    int finTexto = resp.indexOf('\n', finCabecera + 1);
    String texto = resp.substring(finCabecera + 1, finTexto < 0 ? resp.length() : finTexto);
    texto.trim();
    texto.toUpperCase();

    pos = finCabecera + 1;

    Serial.print("\nSMS de ");
    Serial.print(remitente);
    Serial.print(": ");
    Serial.println(texto);

    if (!esNumeroAutorizado(remitente)) {
      Serial.println("-> Numero no autorizado, se ignora.");
    }
    else if (nComandos < MAX_COMANDOS) {
      comandos[nComandos++] = texto;
    }
  }

  // Borrar los mensajes ya leidos ANTES de ejecutar (un comando puede tardar y asi no se
  // procesa dos veces). Los que lleguen despues quedan sin leer y se atienden en la proxima vuelta.
  enviarAT("AT+CMGD=1,1");

  for (int i = 0; i < nComandos; i++) {
    procesarComando(comandos[i]);
  }
}

// Detecta avisos espontaneos del modem (por ahora solo +CMTI = "llego un SMS") sin bloquear.
void escucharModem() {
  while (modem.available()) {
    char c = modem.read();

    if (c == '\n') {
      lineaURC.trim();
      if (lineaURC.startsWith("+CMTI")) {
        Serial.println("\n[Aviso del modem] Llego un SMS nuevo.");
        revisarYa = true;
      }
      lineaURC = "";
    }
    else if (c != '\r') {
      lineaURC += c;
      if (lineaURC.length() > 120) lineaURC = "";   // Proteccion contra basura
    }
  }
}

// ======================= SETUP / LOOP =======================

void setup() {

  pwrkeyReposo();   // Lo primero: dejar el pin de encendido del modem sin pulso

  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  cargarConfig();

  // GPS primero, asi va buscando satelites mientras el modem se configura
  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);
  delay(500);
  gpsAplicarModoConfigurado();

  modem.begin(115200, SERIAL_8N1, RX_MODEM, TX_MODEM);

  // Prender el BG95-M3 (pulso en GPIO5) y esperar a que arranque, y recien despues configurarlo
  encenderModulo();

  while (true) {

    if (esperarModulo()) {

      if (iniciarModulo()) {
        Serial.println("\nBG95-M3 LISTO PARA USAR.");
        break;
      }
    }

    Serial.println("\nFallo durante el inicio.");

    reiniciarModulo();

    delay(5000);
  }

  esperarRed(180000);

  Serial.println("\n===== M.I.C.H.I. LISTO =====");
  Serial.print("Centro: ");
  if (cfg.centroFijado) {
    Serial.print(cfg.lat, 6);
    Serial.print(", ");
    Serial.println(cfg.lon, 6);
  } else {
    Serial.println("sin fijar (mandar SET por SMS)");
  }
  Serial.print("Radio: ");
  Serial.print(cfg.radio, 0);
  Serial.print(" m | Aviso cada ");
  Serial.print(cfg.intervaloMin);
  Serial.println(" min");

  ultimaRevision = millis();
  ultimoChequeo = millis();
  revisarYa = true;   // Revisar de entrada por si quedo algun mensaje pendiente
}

void loop() {

  alimentarGPS();       // Mantener al GPS "al dia"
  escucharModem();      // Ver si el modem aviso algo

  // Guardar el primer fix logrado desde que se encendio
  if (!fixYaRegistrado && gps.location.isValid()) {
    registrarPrimerFix();
  }

  // Comandos por SMS
  if (revisarYa || millis() - ultimaRevision >= INTERVALO_REVISION_MS) {
    revisarYa = false;
    ultimaRevision = millis();
    revisarSMS();
  }

  // Geocerca
  if (millis() - ultimoChequeo >= INTERVALO_CHEQUEO_MS) {
    ultimoChequeo = millis();
    revisarPerimetro();
  }

  // Si el modem dejo de responder varias veces seguidas, lo reiniciamos y reconfiguramos
  if (fallosSeguidos >= FALLOS_PARA_REINICIAR) {
    fallosSeguidos = 0;
    reiniciarModulo();
    iniciarModulo();
    esperarRed(180000);
  }
}

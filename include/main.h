#include <Arduino.h>
#include <Wire.h>
#include "PCF8575.h"
//#include "PCF857X.h" // Required for chip PCF8575
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>




#define PROG_NAME "Panel_Barco"
#define PROG_VER 6
#define PERMISO_SEND 1 //Quitar despues

#define SDA_PIN 18        //remap. Default 21
#define SCL_PIN 5         //remap. Default 22
#define TX0_PIN 1         //Default 1
#define RX0_PIN 3         //Default 3
#define TX1_PIN 21        //remap. Default 10 (usado en SPi Flash)
#define RX1_PIN 22        //remap. Default 9  (usado en SPi Flash)
#define TX2_PIN 19        //Remap. default 17
#define RX2_PIN 23        //Remap. default 16
#define INT0_PIN 34       //(I) Int. pulsos motor 0 (Babor)
#define INT1_PIN 35       //(I) Int. pulsos motor 1 (Estrib.)
#define VBAT_PIN 39       //(I) Detecta si el usuario ha conectado las baterias
#define MOSFET_12V_PIN 32 //Activa mosfet salida tension 12v
#define LIBRE   33  
#define BUZZER_5V_PIN 0   //Sonido teclado
#define BUZZER_12V_PIN 4  //Sonido Alarmas
#define RTC_SQW_PIN 36    //(I) Señal 1Hz del RTC
#define LIBRE_1 25
#define LIBRE_2 26
#define LIBRE_3 27
#define LED1_PIN 12 

/*
  El ESP32 dispone de 3 puertos SPI (FSPI, HSPI y VSPI)
FSPI  (1) //SPI bus attached to the flash (can use the same data lines but different SS)
HSPI  (2) //SPI bus normally mapped to pins 12 - 15, but can be matrixed to any pins
VSPI  (3) //SPI bus normally attached to pins 5, 18, 19 and 23, but can be matrixed to any pins

SD Card Pin	  ESP32 (VSPI interface)	ESP32 (HSPI interface)
    GND	              GND	                      GND
    VCC	           V5 or 3V3	                V5 or 3V3
    MISO	              12	                      19
    MOSI	              13	                      23
    SCK	              14	                      18
    CS	              15	                       5

  Pero se pueden usar los pines que queramos con cada puerto, incluso los pines nativos de VSPI en el HSPI y viceversa
  para ello instanciamos un objeto (cualquier nombre) de la clase SPIClass y le asignamos un puerto (VSPI o HSPI) con:
  SPIClass hspi(HSPI);
  y despues en el setup() inciamos es spi con el objeto anterior y con los pines deseados
  hspi.begin(SCK, MISO, MOSI, CS); //SCK,MISO,MOSI,CS 
  Ahora ya podemos iniciar la SD, con el objeto SD, indicandole el pin CS y el puerto SPI
  SD.begin(CS, hspi)
  Podemos conectar varios dispositivos al mismo puerto SPI, solo tenemos que usar un pin CS diferente para
  cada dispositivo, poniendolo a low para usarlo (los demas pines CS deberan estar a high, pull-up???)

  Tanto el ESP32 como el modulo SD, estan alimentados a 3,3v y no se necesita un adaptador de niveles entre ellos
*/

//SCK,MISO,MOSI,CS (Pines asignados fisicamente al conector SD de la placa TTGO T8 v1.7.1)
#define SD_SCK_PIN 14 //SCK
#define SD_DO_PIN 2   //MISO (master input, slave output)
#define SD_DI_PIN 15  //MOSI
#define SD_CS_PIN 13  //CS (chip select) o SS (slave select)

//Otros defines
#define UART0 0
#define UART1 1
#define UART2 2
#define TMR0 0             //Time 0
#define TMR0_PREESCALER 80 //Freq=80MHz. => 80000000ticks/seg, 80000000/80=1000000ticks/seg, 1Tick=1us
#define TMR0_1seg 1000000  //1tick=1us -> 1*1000000=1000000us=1000ms=1seg
#define TMR0_50ms 50000    //1tick=1us -> 1*50000=50000us=50ms
#define INT_RISING_EDGE true
#define INT_CHANGE false
#define ALARM_AUTORELOAD true

#define EXPANSOR0 0x20 //PCF8575  0100   0   0   0  (7 bits)  //Entradas / Salidas digitales
#define EXPANSOR1 0x21 //PCF8575  0100   0   0   1            //Modulo reles 1
#define EXPANSOR2 0x22 //PCF8575  0100   0   1   0            //Modulo reles 2
#define EXPANSOR3 0x23 //PCF8575  0100   0   1   1            //Modulo reles 3

#define BOTON_ON 1
#define BOTON_OFF 0

#define MT0 0
#define MT1 1
#define RPM_INTERVALO_MOTORES_ms 200 //MT_TMR1_TIME_ms*4      //Cada cuanto tiempo se leeran la rpm de los motores 75ms*4=300ms

//*****  UART  *********************************************************
HardwareSerial Monitor(UART0); //Instanciamos un objeto y le asignamos la UART0
HardwareSerial Panta0(UART1);
HardwareSerial Panta1(UART2);
//O podemos usar los objetos Serial, Serial1 y Serial2, ya instanciados en HardwareSerial.cpp

//*Truco para los mensajes de debug
#define DEBUG true // flag to turn on/off debugging
//Cambia todas las instruciones Monitor.print() por if(DEBUG)Monitor.print(), por lo que solo
//se ejcutaran si DEBUG es true
#define Monitor if(DEBUG)Monitor    
#define Serial Monitor     //Cambia Serial.print() por Monitor.print()

//***** I2C  ***********************************************************
//Para usar un puerto I2C, primero tenemos que instanciar un objeto de la clase TwoWire y
//asignarlo a uno de los 2 puertos I2C (0 ó 1), que tiene el ESP32. En Wire.cpp ya estan
//intanciados estos objetos como TwoWire Wire = TwoWire(0); y TwoWire Wire1 = TwoWire(1);
//Aunque si queremos podemos instanciar nuestros propios objetos con el nombre que queramos.
TwoWire I2C0 = TwoWire(0); //Intanciamos un nuevo objeto y lo asignamos al puero 0
//TwoWire I2C1 = TwoWire(1);  //Intanciamos un nuevo objeto y lo asignamos al puero 1
//Si instanciamos nuestros propios objetos, tendremos dos objetos que coexistiran y tendran
//asignado el mismo puerto, pudiendo usar uno u otro segun convenga.
//En este programa, tengo que usar otros dispositivos I2C, los cuales en sus librerias
//usan el objeto Wire, y por alguna razon no puedo usar Wire con el PCF8575, solo funciona
//si creo un objeto nuevo (I2C0), que apunta al mismo puerto, para usarlo con el PCF8575

//******  PCF8575  ******************************************************
//Para usar la libreria PCF8575, tenemos que instanciar un objeto de la clase PCF8575 y
//asignarle el puerto I2C y la direccion (7 bits), ojo tiene varia sobrecargas
//PCF8575 pcf0_inOut(&I2C0, EXPANSOR0, SDA_PIN, SCL_PIN);  //Entradas/salidas FUNCA
PCF8575 pcf0_inOut(&I2C0, EXPANSOR0); //Entradas/salidas  FUNCA
PCF8575 pcf1_reles(&I2C0, EXPANSOR1); //Placa Reles 1
PCF8575 pcf2_reles(&I2C0, EXPANSOR2); //Placa Reles 2
PCF8575 pcf3_reles(&I2C0, EXPANSOR3); //Placa Reles 3
//Otras sobrecargas:
//PCF8575 pcf8575(0x20);  //addres
//PCF8575 pcf8575(&I2C0, 0x20, 21, 22);   //Objeto I2C, addres, SDA, SCL
//PCF8575 pcf8575(0x20, ARDUINO_UNO_INTERRUPT_PIN, keyPressedOnPCF8575);   //addres, pin, isr

//*****  Reloj RTC  *********
//#define DS3231_ADDR 0x68   //1101 000 (7 bits) -> 0x68  ** No es necesario, la direccion ya
//se encuentra definida en la libreria como #define DS3231_ADDRESS 0x68
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
RTC_DS3231 rtc;

//*****  EEPROM EXTERNA  **********
#define EEPROM_ADDR 0x50  //1010 000 (7 bits) -> 0x50
#define EEPROM_SIZE 65536 //512Kb /8 = 64KB * 1024 = 65536 Bytes
uint8_t EEprom_Init(void);
void writeEEprom(byte deviceaddress, uint16_t eeaddress, byte data);
uint8_t readEEprom(byte deviceaddress, uint16_t eeaddress);
uint16_t EE_graba_datos_defecto(void);
uint16_t EE_leo_datos(void);

//*****  SPI  ***************
//Ahora deberiamos instanciar un objeto (cualquier nombre) de la clase SPIClass y le asignamos un puerto (HSPI o VSPI)
//SPIClass miSPI(HSPI);
//Pero como en SPI.h esta ya instanciado el objeto SPI, con SPIClass SPI(HSPI). Si no hacemos nada ya podemos usar el
//objeto SPI, asignado al puerto HSPI.
SPIClass miSPI(HSPI);

//*****  SD CARD  ***********
File file;

typedef struct
{
   uint8_t status;
   uint8_t tipo;
   uint64_t size;
} SD_STRUCT;

SD_STRUCT sd;

//*****  PROTOTIPOS  ******************************************************************
void scanner_I2C(void);
void pcf0InOut_config(void);
void pcf1Reles_config(void);
void pcf2Reles_config(void);
void pcf3Reles_config(void);
uint8_t DS3232_init(void);
void SD_Init(void);
bool SD_insert(void);
void SD_tipo(void);

void readPanta0();
void readPanta1();
void showDateTime(void);

int ordena_tabla_NX_obj(void);
void desactiva_reles(void);
bool esVisible(int objNum, int page);
bool existe(int objNum, int page);
void print_tabla_NX_obj(void);

int Find_ObjIdx(int ObjNum);
void NX_ObjRcvAccion(int panta, int ObjNum, int val); //Cuando se recibe un comando de la Nextion, actua en consecuencia
void sensor_rmp_accion(bool motor, int pulsos);       //Lee las RPM y actua en consecuencia
void sensor_ADC_accion(int sensor);                   //Lee el sensor correspondiente y actua en consecuencia
void sensor_alarma_accion(int sensor);                //Accion cuando se activa una alarma

void NX_actualiza_datos_pagina(int page); //Actualiza todos los datos estaticos de una pagina, cuando se carga una nueva
void NX_send_obj(int panta, int ObjNum, int atr, int val, bool forzar);
//void NX_send_obj(int panta, int ObjNum, char *txt);

void NX_confirmar_boton(int objIdx, int val);

//void rele_set(int rele, bool estado);     //Obsoleta
void rele_set(uint8_t rele, bool estado);
void salida_set(uint8_t salida, bool estado);
//int map(int valor, int entradaMin, int entradaMax, int salidaMin, int salidaMax);
//int map(int valor, int entradaMin, int entradaMax, int salidaMin, int salidaMax);
float map(float valor, float entradaMin, float entradaMax, float salidaMin, float salidaMax);
int round_mult(int valor, int multiplo);
void buzzer_asincrono(int repet, int time_on, int time_off);
void buzzer(int repet, int time_on, int time_off);

void flaps_control(int ObjNum, int val);

void NX_cambio_pagina(int panta, int page); //No la estoy usando
void startStop(bool motor, int objStart);
void NX_solicita_pagina(bool panta, bool forzar); //Solicita a la Nextion que envie el numero de pagina actual, No espera por la respuesta
void NX_getPage(bool panta, bool forzar);         //Solicita a la Nextion que envie el numero de pagina actual, y espera un tiempo por la respuesta
void NX_send_cmd(int panta, int objNum, int cmd, int val);
bool ADS1115_ready(int modulo);
int ads1115_read(int modulo, int reg);
bool ads1115_write(int modulo, int reg, int dat);
bool ADS1115_init(int modulo);
int ADS1115_raw_measure(int modulo, int canal);
bool ADS1115_set_canal(int canal, bool ini_conver);
int ADS1115_read_adc(int canal, bool esperar);

//*******  PROTOTIPOS SD CARD  *************
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void printDirectory(File dir, int numTabs);

//Numero de pagina de la Nextion
//      0       1        2        3           4          5       6        7
enum{ PMAIN, PMOTOR0, PMOTOR1, PMOTOR0Y1, PINTERRUP, PCONFIG, PRELES, PPRUEBAS};

#define NX0 0           //Panta 0
#define NX1 1           //Panta 1
#define NXALL 0xFE      //(254)Todas las pantallas
#define NXNULL 0XFF     //Pantalla sin asignar
#define NXPAGENULL 0xFF //Pagina sin asignar

uint32_t current_millis, previus_millis;
uint8_t time5seg;

//bool gNX_premiso_send=1;
int gNX_page[2] = {NXPAGENULL, NXPAGENULL}; //almacena el numero de pagina actual de cada pantalla. La pantalla es el indice

char gRX_trama[2][5]; //Almacena el comando (4 bytes) enviados por ambas pantallas (0,1)
enum{TRAMA_INI, TRAMA_OBJNUM, TRAMA_VAL, TRAMA_FIN}; //Indice del parametro recibido en gRX_trama[]
bool flag_rda1_int = 0;
bool flag_rda2_int = 0;

int gRPM_pulsos_cont[2] = {0, 0}; //Pulsos de la rfonica de cada motor
//bool dLuz=0;
bool flag_tmr1_time = 0;
bool flag_buzzer_activo = 0;
bool flag_tmr1_1000ms = 0;

int rpm_rfonica_dientes = 33; //Por defecto.
int tmr1_offset = 0;          //Cambiar a int
bool flag_MT_sensor_time = 0; //Si true, indica que hay que leer algun sensor del motor
bool MT_motor_num = 0;        //Motor 0->lado izq. y motor 1->lado dcho.


//bool flag_alarma=0;
//bool flag_alarmaSonido=0;

//Pongo el bit como modificado en todos los sensores. necesario para la primerera vez
//bool sensorAlarma[9][2]={1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0}; //modificado,estado. Tipo bool solo 3bytes=21bits (funca)
int sensorAlarma_estado[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //estado. Tipo bool solo 2bytes. El primero no es un sensor, es NX_FINDPAGE

//      0      1        2        3       4       5         6          7          8         9=Este no es un sensor, indica el valor del ultimo item
//enum{ALTEMP0,ALPRES0,ALALTER0,ALTEMP1,ALPRES1,ALALTER1,ALACHIQUE1,ALACHIQUE2,ALACHIQUE3,ALULTIMO};
//enum{ALTEMP0,ALPRES0,ALALTER0,ALTEMP1,ALPRES1,ALALTER1,ALULTIMO};

int millis_ant = 0;        //Almacena los millis usados en el bucle principal
int buzzer_millis_ant = 0; //Almacena los millis usados en la funcion buzzer

bool flag_time_start[2] = {0, 0};
int motor_estado[2] = {0, 0}; //Los 2 motores estado stop. Actualizar al estado real, para que no se paren ????
int tiempo_boton_start[2];    //tiempo boton start/stop pulsado
//bool flag_start_permiso_soltar_boton[2]={1,1};

bool flag_bateria_on = 0;

//EEPROM
//#define EE_Contacto1 10
//#define EE_Start1    11
//#define EE_flapL     12

//Aqui debo incluir el numero de objeto de todos los objetos de la tabla NX_obj[].
//Procedimiento: Crear un objeto en la Nextion y asignarle un nombre.
//En el programa CCS C, crear un #define con ese mismo nombre anteponiendo 'obj' y asignarle un numero secuencial correlativo, sin huecos, este sera el id del objeto
//Crear en la struc NX_obj[] una linea con ese mismo nombre y los parametros adecuados. Da igual la posicion, se ordenara al iniciarse el programa.
//Si hubiera que eliminar un objeto, no borrar la linea, borrar el nombre del objeto, pero sustituirlo por el numero que representaba, el resto de
//parametros ponerlos en 0
//En la Nextion, en el codigo de inicio, crear una variable int con el mismo nombre e ID anterior
#define objPPage 0
#define objCContacto1 1
#define objCStart1 2
#define objCStop1 3
#define objCPilotoAuto 4
#define objCAguaSalada 5
#define objCAguaDulce 6
#define objCFlap1UP 7
#define objCFlap1DW 8
#define objCFlap2UP 9
#define objCFlap2DW 10
#define objCContacto2 11
#define objCStart2 12
#define objCStop2 13
#define objCClaxon 14
#define objCLimpia1 15
#define objCLimpia2 16
#define objCLimpia3 17
#define objCLimpAgua 18
#define objCFilar 19
#define objCVirar 20
#define objCAchique1 71
#define objCAchique2 72
#define objCAchique3 73
#define objCLuzPos 77
#define objCLuzFondeo 79
#define objCFaroPirata 81
#define objCLuzCabina 83
#define objCLuzCamarote 85
#define objCLuzAseo 87
#define objCLuzBanera 89
#define objCLibre12 91
#define objCLibre13 92
#define objCLibre14 93
#define objCLibre15 94
#define objCLibre16 95
#define objCLibre27 96
#define objCLibre28 97
#define objCLibre29 98
#define objCLibre30 99
#define objCLibre31 100
#define objCLibre32 101
#define objCLibre42 102
#define objCLibre43 103
#define objCLibre44 104
#define objCLibre45 105
#define objCLibre46 106
#define objCLibre47 107
#define objCLibre48 108

#define objaRpm0 21
#define objnRpm0 22
#define objhRpm0 23
#define objpTemp0 24
#define objaTemp0 25
#define objnTemp0 26
#define objhTemp0 27
#define objpAlTemp0 117
#define objaPres0 28
#define objnPres0 29
#define objhPres0 30
#define objpAlPres0 118
#define objaAlter0 31
#define objfAlter0 32
#define objhAlter0 33
#define objpAlAlter0 119
#define objaFuel 34
#define objnFuel 35
#define objhFuel 36
#define objaRpm1 37
#define objnRpm1 38
#define objhRpm1 39
#define objpTemp1 40
#define objaTemp1 41
#define objnTemp1 42
#define objhTemp1 43
#define objpAlTemp1 120
#define objaPres1 44
#define objnPres1 45
#define objhPres1 46
#define objpAlPres1 121
#define objaAlter1 47
#define objfAlter1 48
#define objhAlter1 49
#define objpAlAlter1 122
#define objfBat1 50
#define objfBat2 51
#define objfBat3 52

#define objbEnter 53
#define objpStart0 54
#define objpStart1 55
#define objbClaxon 56
#define objdPilotoAuto 57
#define objdAguaSalada 58
#define objdAguaDulce 59
#define objbFilar 60
#define objbVirar 61
#define objbFlapsDw 62
#define objbFlapsUp 63
#define objbFlapsL 64
#define objbFlapsR 65
#define objbFlaps0 66
#define objdLimpia1 67
#define objdLimpia2 68
#define objdLimpia3 69
#define objdLimpAgua 70
#define objbLimpiaBDw 109
#define objbLimpiaBUp 110
#define objbLimpiaCDw 111
#define objbLimpiaCUp 112
#define objbLimpiaEDw 113
#define objbLimpiaEUp 114
#define objbLimpAguaDw 115
#define objbLimpAguaUp 116
#define objbAchique1 74
#define objbAchique2 75
#define objbAchique3 76
#define objdLuzPos 78
#define objdLuzFondeo 80
#define objdFaroPirata 82
#define objdLuzCabina 84
#define objdLuzCamarote 86
#define objdLuzAseo 88
#define objdLuzBanera 90
#define objbResetPic 123 //Ultimo

#define OBJ_EOF 0xFF //No encontrado (End Of File)
#define NOEEP 0      //No guardado en la eeprom
#define NOAUX 0xFF   //Campo auxiliar no usado

enum{ATRVAL, ATRTXT, ATRPIC, ATRPCO, NOATTR = 0xFF};
//atrib_cod                // 0     1     2     3
char NX_obj_atrib_txt[][4] = {"val", "txt", "pic", "pco"}; //Texto del Atributo que quiero cambiar del objeto de la NX

typedef struct
{
   int page;
   int vis; //Probar con bool
} NX_ID_STRUCT;

#define RELE_REP 3 //Rl0, Rl1, Rl3
#define PAGE_REP 3 //Page[0], Page[1], Page[2]

typedef struct
{
   uint8_t objNum;
   char tipoN;         //tipo objeto Nextion
   uint8_t tipo;     //Tipo: Pagina='P', Almacen ='A', Salida='S', Boton='B'
   char objName[13]; //12+1 Nombre unico del objeto, puede repetirse en distinta page
   uint8_t atrib_cod;    //codigo del atributo del objeto que voy a cambiar de la pantalla NX
   int16_t val_ant;
   uint8_t confir[2]; //Si hay que confirmar el boton tanto si se ha presionado co soltado
   uint16_t EEpos;
   uint8_t estado;                  //Estado del pulsador o accion que actuara sobre 1 o mas reles. NO se refiere al estado de un rele, sino al estado del boton
   NX_ID_STRUCT page[PAGE_REP]; //NX_obj[1].Page[0].page
   uint8_t act;                     //Como se actualizan los objetos en la Nextion. 0=No, 1=Al cargar la pagina,  2=Timer(solo la pagina activa)
   uint8_t aux1;
   uint8_t aux2;     //Indice tabla auxiliar, la tabla puede ser diferente para cada tipo de objeto
} NX_OBJ_STRUCT; //*** ojo: si cambio algo, tengo que cambiarlo tambien en ordena_tabla_NX_obj()

enum { TPAGE, TACTUA, TSAL, TPULSA};     //Tipo objeto: Page, Actuador, Salida, Pulsador 

/*
   La tabla NX_obj[], contiene todos los objetos de la Nextion que tengan que interactuar com el PIC, teniendo en cuenta que el numero de objeto corresponde
con el indice de la tabla. No es un problema si en esta declaracion no estan ordenados, se ordenaran al inicio mediante el metodo de la burbuja. Si al
ordenarse hubieran huecos, se meteria un linea con todo ceros, para mantener la relacion del ojbNum con el indice. 
   En la tabla estan todas la propiedades (constantes) y valores (variables) de un objeto, que indican el comportamiento y como interactuar con el PIC.
   En esta declaracion de la tabla, se le asignan propiedades (constantes) que no cambiaran y valores (variables) que son los valores inciales y que al inicio
del programa seran actualizados desde la eeprom. Ademas estos datos variables deberan guardarse en la eeprom cada vez que cambien, para que mantengan sus 
   valores entre reinicios.
   IDEA: para ahorrar memoria puedo asignar en la struct que las variables sean de uno o varios bits, segun se necesite
   Nota: Una mejora para la pantalla Nextion, es que para los numeros de objeto en vez de ponerlos directamente en cada objeto se puede crear una variable
de sistema y asignarle el numero correspondiente, y en el objeto se pone esta variable. Solo tengo que cambiar el valor de esta variable y se cmabiara 
en toda los sitios de la Nextion.
   Nota:El valor ID de los objetos de la Nextion no es utilizable, ya que si borro un objeto, cambiara el ID de todos los objetos que sean mayores
   IDEA:Para hacer la tabla mas pequeña, puedo sacar algunos datos y ponerlos en structuras independientes, pero tendria el problema del indice de esas
nuevas tablas, una solucion seria hacer una funcion que me devuelva el indice correspondiente a ese objeto. Otra solucion seria añadir a cada objeto de
la tabla NX_obj[], un campo que contenga el indice de la otra tabla donde estan los datos, seria interesante para los reles, la visibilidad de las paginas...etc
otra solucion es añadir el indice como campo de la tabla y ordenarla por porgrama segun ese indice.
*/

//                                                -codigo del atributo del objeto que voy a cambiar de la pantalla NX ("val","txt","pic","pco"). No se si hace falta?????????
//                                               /       -Valor anterior.
//                                               |      /       -Confirmar boton ON
//                                               |      |      /        -Confirmar boton OFF
//                                               |      |      |       /     -Pos de la eeprom (10-1023) donde se guardara el dato   0=No se guarda
//                                               |      |      |       |    /      -Estado del pulsador o accion que actuara sobre 1 o mas reles.               -
//                                               |      |      |       |    |     /      -Numero de pagina donde existe
//                                               |      |      |       |    |     |     /        -Indica si el objeto esta visible.
//                                               |      |      |       |    |     |     |       /                               -Como se actualizan los objetos en la Nextion. 0=No, 1=Al cargar la pagina o si hay algun cambio,  2=Timer(solo la pagina activa)
//                                               |      |      |       |    |     |     |       |                              /     -Indice tabla auxiliar, la tabla puede ser diferente para cada tipo de objeto
//                                               |      |      |       |    |     |     |       |                              |    /
//    ObjNum,     tipoN,  tipo,   objname,     atrib, vant,confON,confOFF,EEpos,estado, pag,   vis,  pag,   vis,  pag,  vis,  act. aux1,  aux2  
//                objNum
NX_OBJ_STRUCT NX_obj[] = {//                                  conf  conf
//    ObjNum,     tipoN,  tipo,     objname,     atrib, vant,  ON   OFF  EEpos,estado, pag,    vis,  pag,   vis,  pag,  vis,  act. aux1,  aux2                                                                                                                                        //rele
    objPPage,       'P', TPAGE,    "Ppage",        0,    0,    0,    0,NOEEP,     0,      0,    0,    1,     0,    0,    0,    0, NOAUX, NOAUX,  //NOAUX=No usado, 0=Rele sin asignar
    objCContacto1,  'C', TACTUA,   "CContacto1",   0,    0,    0,    1,   10,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     1, //Numero de rele
    objCStart1,     'C', TACTUA,   "CStart1",      0,    0,    0,    1,   11,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     2,
    objCStop1,      'C', TACTUA,   "CStop1",       0,    0,    0,    1,   12,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     3,
    objCPilotoAuto, 'C', TACTUA,   "CPilotoAuto",  0,    0,    0,    1,   13,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     5,
    objCAguaSalada, 'C', TACTUA,   "CAguaSalada",  0,    0,    0,    1,   14,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     6,
    objCAguaDulce,  'C', TACTUA,   "CAguaDulce",   0,    0,    0,    1,   15,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     7,
    objCFlap1UP,    'C', TACTUA,   "CFlap1UP",     0,    0,    0,    1,   16,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     8,
    objCFlap1DW,    'C', TACTUA,   "CFlap1DW",     0,    0,    0,    1,   17,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     9,
    objCFlap2UP,    'C', TACTUA,   "CFlap2UP",     0,    0,    0,    1,   18,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    10,
    objCFlap2DW,    'C', TACTUA,   "CFlap2DW",     0,    0,    0,    1,   19,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    11,
    objCAchique1,   'C', TACTUA,   "CAchique1",    0,    0,    0,    1,   20,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    24,
    objCAchique2,   'C', TACTUA,   "CAchique2",    0,    0,    0,    1,   21,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    25,
    objCAchique3,   'C', TACTUA,   "CAchique3",    0,    0,    0,    1,   22,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    26,
    objCContacto2,  'C', TACTUA,   "CContacto2",   0,    0,    0,    1,   23,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    17,
    objCStart2,     'C', TACTUA,   "CStart2",      0,    0,    0,    1,   24,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    18,
    objCStop2,      'C', TACTUA,   "CStop2",       0,    0,    0,    1,   25,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    19,
    objCClaxon,     'C', TACTUA,   "CClaxon",      0,    0,    0,    1,   26,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     4,
    objCLimpia1,    'C', TACTUA,   "CLimpia1",     0,    0,    0,    1,   27,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    20,
    objCLimpia2,    'C', TACTUA,   "CLimpia2",     0,    0,    0,    1,   28,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    21,
    objCLimpia3,    'C', TACTUA,   "CLimpia3",     0,    0,    0,    1,   29,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    22,
    objCLimpAgua,   'C', TACTUA,   "CLimpAgua",    0,    0,    0,    1,   30,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    23,
    objCFilar,      'C', TACTUA,   "CFilar",       0,    0,    0,    1,   31,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    40,
    objCVirar,      'C', TACTUA,   "CVirar",       0,    0,    0,    1,   32,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    41,
    objCLuzPos,     'C', TACTUA,   "CLuzPos",      0,    0,    0,    1,   33,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    37,
    objCLuzFondeo,  'C', TACTUA,   "CLuzFondeo",   0,    0,    0,    1,   34,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    38,
    objCFaroPirata, 'C', TACTUA,   "CFaroPirata",  0,    0,    0,    1,   35,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    39,
    objCLuzCabina,  'C', TACTUA,   "CLuzCabina",   0,    0,    0,    1,   36,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    33,
    objCLuzCamarote,'C', TACTUA,   "CLuzCamarote", 0,    0,    0,    1,   37,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    34,
    objCLuzAseo,    'C', TACTUA,   "CLuzAseo",     0,    0,    0,    1,   38,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    35,
    objCLuzBanera,  'C', TACTUA,   "CLuzBanera",   0,    0,    0,    1,   39,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,    36, //rele
    objCLibre12,    'C', TACTUA,   "CLibre12",     0,    0,    0,    1,   40,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,      //0=Rele sin asignar
    objCLibre13,    'C', TACTUA,   "CLibre13",     0,    0,    0,    1,   41,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre14,    'C', TACTUA,   "CLibre14",     0,    0,    0,    1,   42,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre15,    'C', TACTUA,   "CLibre15",     0,    0,    0,    1,   43,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre16,    'C', TACTUA,   "CLibre16",     0,    0,    0,    1,   44,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre27,    'C', TACTUA,   "CLibre27",     0,    0,    0,    1,   45,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre28,    'C', TACTUA,   "CLibre28",     0,    0,    0,    1,   46,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre29,    'C', TACTUA,   "CLibre29",     0,    0,    0,    1,   47,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre30,    'C', TACTUA,   "CLibre30",     0,    0,    0,    1,   48,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre31,    'C', TACTUA,   "CLibre31",     0,    0,    0,    1,   49,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre32,    'C', TACTUA,   "CLibre32",     0,    0,    0,    1,   50,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre42,    'C', TACTUA,   "CLibre42",     0,    0,    0,    1,   51,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre43,    'C', TACTUA,   "CLibre43",     0,    0,    0,    1,   52,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre44,    'C', TACTUA,   "CLibre44",     0,    0,    0,    1,   53,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre45,    'C', TACTUA,   "CLibre45",     0,    0,    0,    1,   54,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre46,    'C', TACTUA,   "CLibre46",     0,    0,    0,    1,   55,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre47,    'C', TACTUA,   "CLibre47",     0,    0,    0,    1,   56,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
    objCLibre48,    'C', TACTUA,   "CLibre48",     0,    0,    0,    1,   57,     0, PRELES,    1,    0,     0,    0,    0,    1, NOAUX,     0,
//    ObjNum,     tipoN,  tipo,     objname,     atrib, vant,confON,confOFF,EEpos,estado, pag,   vis,  pag,   vis,  pag,  vis,  act. aux1,  aux2 
    objaRpm0,       'a', TSAL,   "aRpm0",          0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objnRpm0,       'n', TSAL,   "nRpm0",          0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhRpm0,       'h', TSAL,   "hRpm0",          0,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    0, NOAUX, NOAUX, //No existe el objeto??, la pongo como no visible
    objpTemp0,      'p', TSAL,   "pTemp0",         0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objaTemp0,      'a', TSAL,   "aTemp0",         0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objnTemp0,      'f', TSAL,   "nTemp0",         0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhTemp0,      'h', TSAL,   "hTemp0",         0,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objpAlTemp0,    'p', TSAL,   "pAlTemp0",  ATRPIC,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    1, NOAUX, NOAUX,
    objaPres0,      'a', TSAL,   "aPres0",         0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objnPres0,      'n', TSAL,   "nPres0",         0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhPres0,      'h', TSAL,   "hPres0",    NOATTR,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objpAlPres0,    'p', TSAL,   "pAlPres0",  ATRPIC,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    1, NOAUX, NOAUX,
    objaAlter0,     'a', TSAL,   "aAlter0",        0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objfAlter0,     'f', TSAL,   "fAlter0",        0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhAlter0,     'h', TSAL,   "hAlter0",   NOATTR,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objpAlAlter0,   'p', TSAL,   "pAlAlter0", ATRPIC,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    1, NOAUX, NOAUX,
    objaFuel,       'a', TSAL,   "aFuel",          0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objnFuel,       'n', TSAL,   "nFuel",          0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhFuel,       'h', TSAL,   "hFuel",     NOATTR,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    0, NOAUX, NOAUX,
//    ObjNum,     tipoN,  tipo,   objname,     atrib, vant,confON,confOFF,EEpos,estado, pag,   vis,  pag,   vis,  pag,  vis,  act. aux1,  aux2 
    objaRpm1,       'a', TSAL,   "aRpm1",          0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objnRpm1,       'n', TSAL,   "nRpm1",          0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhRpm1,       'h', TSAL,   "hRpm1",          0,    0,    0,    0,    0,     0, PMOTOR1,   0,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objpTemp1,      'p', TSAL,   "pTemp1",         0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objaTemp1,      'a', TSAL,   "aTemp1",         0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objnTemp1,      'f', TSAL,   "nTemp1",         0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhTemp1,      'h', TSAL,   "hTemp1",    NOATTR,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objpAlTemp1,    'p', TSAL,   "pAlTemp1",  ATRPIC,    0,    0,    0,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    1, NOAUX, NOAUX,
    objaPres1,      'a', TSAL,   "aPres1",         0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objnPres1,      'n', TSAL,   "nPres1",         0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhPres1,      'h', TSAL,   "hPres1",    NOATTR,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objpAlPres1,    'p', TSAL,   "pAlpres1",  ATRPIC,    0,    0,    0,    0,     0, PMOTOR1,   0,    0,     0,    0,    0,    1, NOAUX, NOAUX,
    objaAlter1,     'a', TSAL,   "aAlter1",        0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objfAlter1,     'f', TSAL,   "fAlter1",        0,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objhAlter1,     'h', TSAL,   "hAlter1",   NOATTR,    0,    0,    0,    0,     0, PMOTOR1,   1,    0,     0,    0,    0,    0, NOAUX, NOAUX,
    objpAlAlter1,   'p', TSAL,    "pAlAlter1", ATRPIC,   0,    0,    0,    0,     0, PMOTOR1,   0,    0,     0,    0,    0,    1, NOAUX, NOAUX,
    objfBat1,       'f', TSAL,   "fBat1",          0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objfBat2,       'f', TSAL,   "fBat2",          0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
    objfBat3,       'f', TSAL,   "fBat3",          0,    0,    0,    0,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    2, NOAUX, NOAUX,
//    ObjNum,     tipoN,  tipo,   objname,     atrib, vant,confON,confOFF,EEpos,estado, pag,   vis,  pag,   vis,  pag,  vis,  act. aux1,  aux2   idx tabla Pulsador_Rele[]
    objbEnter,      'b', TPULSA, "bEnter",         0,    0,    1,    0,    0,     0, PMAIN,     1,    0,     0,    0,    0,    1, NOAUX, NOAUX,
    objpStart0,     'b', TPULSA, "pStart0",   ATRPIC,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    0,     0,     0, //aux1=idx Picture[]
    objpStart1,     'b', TPULSA, "pStart1",   ATRPIC,    0,    1,    1,    0,     0, PMOTOR0,   0,    0,     0,    0,    0,    0,     0,     1, //aux1=idx Picture[]
    objbClaxon,     'b', TPULSA, "bClaxon",        0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    1, NOAUX,     2,
    objdPilotoAuto, 'd', TPULSA, "dPilotoAuto",    0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,     3,
    objdAguaSalada, 'd', TPULSA, "dAguaSalada",    0,    0,    1,    1,    0,     0, PMOTOR0,   1,    4,     1,    0,    0,    1, NOAUX,     4,
    objdAguaDulce,  'd', TPULSA, "dAguaDulce",     0,    0,    1,    1,    0,     0, PMOTOR0,   1,    4,     1,    0,    0,    1, NOAUX,     5,
    objbFilar,      'b', TPULSA, "bFilar",         0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,     6,
    objbVirar,      'b', TPULSA, "bVirar",         0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,     7,
    objbFlapsDw,    'b', TPULSA, "bFlapsDw",       0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,     8,
    objbFlapsUp,    'b', TPULSA, "bFlapsUp",       0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,     9,  //Aux2=indice tabla Pulsador
    objbFlapsL,     'b', TPULSA, "bFlapsL",        0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    10,
    objbFlapsR,     'b', TPULSA, "bFlapsR",        0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    11,
    objbFlaps0,     'b', TPULSA, "bFlaps0",        0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    12,
    objdLimpia1,    'd', TPULSA, "dLimpia1",       0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    13,
    objdLimpia2,    'd', TPULSA, "dLimpia2",       0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    14,
    objdLimpia3,    'd', TPULSA, "dLimpia3",       0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    15,
    objdLimpAgua,   'd', TPULSA, "dLimpAgua",      0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    16,
    objbLimpiaBDw,  'b', TPULSA, "bLimpiaBDw",     0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX,   //Solo actuara sobre la logica del programa
    objbLimpiaBUp,  'b', TPULSA, "bLimpiaBUp",     0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX,   //Solo actuara sobre la logica del programa
    objbLimpiaCDw,  'b', TPULSA, "bLimpiaCDw",     0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX,   //Solo actuara sobre la logica del programa
    objbLimpiaCUp,  'b', TPULSA, "bLimpiaCUp",     0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX,   //Solo actuara sobre la logica del programa
    objbLimpiaEDw,  'b', TPULSA, "bLimpiaEDw",     0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX,   //Solo actuara sobre la logica del programa
    objbLimpiaEUp,  'b', TPULSA, "bLimpiaEUp",     0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX,   //Solo actuara sobre la logica del programa
    objbLimpAguaDw, 'b', TPULSA, "bLimpAguaDw",    0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX, //Solo actuara sobre la logica del programa
    objbLimpAguaUp, 'b', TPULSA, "bLimpAguaUp",    0,    0,    1,    1,    0,     0, PMOTOR0,   1,    2,     1,    0,    0,    0, NOAUX, NOAUX, //Solo actuara sobre la logica del programa
    objbAchique1,   'b', TPULSA, "bAchique1",      0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    17,
    objbAchique2,   'b', TPULSA, "bAchique2",      0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    18,
    objbAchique3,   'b', TPULSA, "bAchique3",      0,    0,    1,    1,    0,     0, PMOTOR0,   1,    0,     0,    0,    0,    1, NOAUX,    19,
    objdLuzPos,     'd', TPULSA, "dLuzPos",        0,    0,    1,    1,    0,     0, PMOTOR0,   1, PINTERRUP,1,    0,    0,    1, NOAUX,    20,
    objdLuzFondeo,  'd', TPULSA, "dLuzFondeo",     0,    0,    1,    1,    0,     0, PMOTOR0,   1, PINTERRUP,1,    0,    0,    1, NOAUX,    21,
    objdFaroPirata, 'd', TPULSA, "dFaroPirata",    0,    0,    1,    1,    0,     0, PINTERRUP, 1,    0,     0,    0,    0,    1, NOAUX,    22,
    objdLuzCabina,  'd', TPULSA, "dLuzCabina",     0,    0,    1,    1,    0,     0, PINTERRUP, 1,    0,     0,    0,    0,    1, NOAUX,    23,
    objdLuzCamarote,'d', TPULSA, "dLuzCamarote",   0,    0,    1,    1,    0,     0, PINTERRUP, 1,    0,     0,    0,    0,    1, NOAUX,    24,
    objdLuzAseo,    'd', TPULSA, "dLuzAseo",       0,    0,    1,    1,    0,     0, PINTERRUP, 1,    0,     0,    0,    0,    1, NOAUX,    25,
    objdLuzBanera,  'd', TPULSA, "dLuzBanera",     0,    0,    1,    1,    0,     0, PINTERRUP, 1,    0,     0,    0,    0,    1, NOAUX,    26,
    objbResetPic,   'b', TPULSA, "bResetPic",      0,    0,    1,    1,    0,     0, PCONFIG,   1,    0,     0,    0,    0,    1, NOAUX, NOAUX
//    ObjNum,     tipoN,  tipo,   objname,     atrib, vant,confON,confOFF,EEpos,estado, pag,   vis,  pag,   vis,  pag,  vis,  act. aux1,  aux2 
};

//**** REVISAR PROCESO DE ORDENACION

//Si cambio el ObjNum de algun objeto que envie datos desde la Nextion, debo cambiarlo tambien en la nextion
//Puedo cambiar los objetos de sitio en la tabla, y tambien añadir los que quiera, PERO si borro alguno, debe
//mantenerse la linea, aunque sea con valores todo cero. El mayor numero de ObjNum debe de ser igual al numero
//de lineas de la tabla, por que sino fallara el proceso de indexacion

//Comandos para la Nextion usados en NX_send_cmdx(). eje:vis pAlTemp0,0
//   0        1      2       3       4
enum {CMDREST, CMDPAGE, CMDCLICK, CMDVIS, CMDTSW};
//  0       1      2      3     4
char cmd2txt[][6] = {"rest", "page", "click", "vis", "tsw"};

/*
   Para manejar los reles, tenemos que definir el termino pulsador, que puede ser un boton, un interruptor, una
orden del PIC...etc. y el termino actuador, que puede ser un led, un motor, un hidraulico...etc.
   Cuando se activa el pulsador, buscara todos los reles asociados, y cada actuador tendra asignado el
correspondiente rele.

En el objeto pulsador de la tabla NX_obj, hay un campo multifuncion (aux) que puede tener diferenes significados:
-Si es un objeto tipo 'C' (combo), representa el numero de rele asociado a ese actuador
-Si es un objeto tipo 'b' o 'd' (pulsador) representa el indice de la tabla fija Pulsador_Rele[][] , donde estan
los objetos de tipo 'C' de la tabla NX_obj[] que continenen los reles que manejara ese pulsador. 
*/

//#define NOPIC 0 //No picture
#define PICTURE_CAMPOS 3 //Numero de campos por registro de la tabla Pictures[] ?????
#define PICTURE_MAX 3    //??????

#define MT_STOP_PIC 18 //id picture de la Nextion
#define MT_CONTACT_PIC 19
#define MT_START_PIC 20

//Esta tabla es fija, no se puede modificar por programa. Si añado o cambio algun indice debo reflejarlo en la tabla
//NX_obj[] en la parte de los objetos tipo 'b' o 'd'
//IDEA: si añado un indice numerico como parte de la tabla, la podre ordenar
// aux1                  picture0,    picture1,       picture2
const int Pictures[][PICTURE_MAX] = {
    MT_STOP_PIC, MT_CONTACT_PIC, MT_START_PIC //El indice debe mantenerse
};

#define NORELE 0 //No rele
#define PULSA_RELES_MAX 3

//Esta tabla es fija, no se puede modificar por programa. Si añado o cambio algun indice debo reflejarlo en la tabla
//NX_obj[] en la parte de los objetos tipo 'b' o 'd'
//IDEA: si añado un indice numerico como parte de la tabla, la podre ordenar
//Aqui obtengo el objeto (idx) de la tabla NX_obj[] que contiene el numero de rele, hasta 3 objetos
//rele=Nx_obj[idx].aux2
// aux2                       objRele1,        objRele2,      objRele3
const int Pulsador_Actuador[][PULSA_RELES_MAX] = {
    objCContacto1, objCStart1, objCStop1, //0 El indice debe mantenerse
    objCContacto2, objCStart2, objCStop2, //1
    objCClaxon, NORELE, NORELE,           //2
    objCPilotoAuto, NORELE, NORELE,       //3
    objCAguaSalada, NORELE, NORELE,       //4
    objCAguaDulce, NORELE, NORELE,        //5
    objCFilar, NORELE, NORELE,            //6
    objCVirar, NORELE, NORELE,            //7
    objCFlap1DW, objCFlap2DW, NORELE,     //8
    objCFlap1UP, objCFlap2UP, NORELE,     //9
    objCFlap1UP, objCFlap2DW, NORELE,     //10
    objCFlap1DW, objCFlap2UP, NORELE,     //11
    objCFlap1UP, objCFlap2UP, NORELE,     //12
    objCLimpia1, NORELE, NORELE,          //13
    objCLimpia2, NORELE, NORELE,          //14
    objCLimpia3, NORELE, NORELE,          //15
    objCLimpAgua, NORELE, NORELE,         //16
    objCAchique1, NORELE, NORELE,         //17
    objCAchique2, NORELE, NORELE,         //18
    objCAchique3, NORELE, NORELE,         //19
    objCLuzPos, NORELE, NORELE,           //20
    objCLuzFondeo, NORELE, NORELE,        //21
    objCFaroPirata, NORELE, NORELE,       //22
    objCLuzCabina, NORELE, NORELE,        //23
    objCLuzCamarote, NORELE, NORELE,      //24
    objCLuzAseo, NORELE, NORELE,          //25
    objCLuzBanera, NORELE, NORELE         //26
};

#define NOEXPAN 0xFF
//A2  A1  A0
//MUY IMPORTANTE: al expansor 0 ponerle   0   0   0
//                            1           0   0   1
//                            2           0   1   0
// y asi sucesivamente. Osea el valor de esta parte de la direccion i2c debe de coincidir con el indice de la estructura Expan[]

/*
typedef struct
{
   int expan; //Expansor donde esta conectado (0-254), si 0xFF NO expansor, conectado directamente al PIC
   int bit;   //Bit del expansor. Cambiar a 8 bits
   //int estado;      //Estado (ON/OFF), el estado cambiara segun el estado guardado en la eeprom
} EXPAN2RELE_STRUCT;

#define PIN_D0 0 //Quitar \
                 //rele_num       expan,   bit,
EXPAN2RELE_STRUCT Rele_Expansor[] = {
    NOEXPAN, PIN_D0, //0,    //Rele_num=0, NO expansor, pin conectado directamente al PIC Quitar si se puede
    0, 14,           //Rele_num=1, expansor 0, bit 0
    0, 15,           //Rele_num=2, expansor 0, bit 1
    0, 12,
    0, 13,
    0, 10,
    0, 11,
    0, 8,
    0, 9, //Rele_num=8
    0, 6, //Rele_num=9
    0, 7,
    0, 4,
    0, 5,
    0, 2,
    0, 3,
    0, 0,
    0, 1, //Rele_num=16

    1, 0, //Rele_num=17
    1, 1,
    1, 2,
    1, 3,
    1, 4,
    1, 5,
    1, 6,
    1, 7, //Rele_num=24
    1, 8, //Rele_num=25
    1, 9,
    1, 10,
    1, 11,
    1, 12,
    1, 13,
    1, 14,
    1, 15 //Rele_num=32
};

//ADDR  A2  A1  A0
typedef struct
{
   int addr;   //Direccion segun el tipo de expansor
   int bits;   //Tamaño del expansor 8 o 16 bits
   int byt[2]; //palabra que se envia a cada expansor con el estado de cada rele
   //Ojo algunos expansores son de 16 bits
} EXPAN_STRUCT;

//expansor   ADDR     bits  byte0 byte1            //Tantos como expansores conecte
EXPAN_STRUCT tExpansor[] = {
    EXPANSOR0, 16, 0xFF, 0xFF, //Expansor 0,  0xFF todo desactivado (logica inversa)
    EXPANSOR1, 16, 0xFF, 0xFF, //Expansor 1,  0xFF todo desactivado (logica inversa)
    EXPANSOR2, 16, 0xFF, 0xFF, //Expansor 2
    EXPANSOR3, 16, 0xFF, 0xFF, //Expansor 3
};
*/
//Nuevo
//Esta tabla me relaciona el numero de rele con el pin del expansor correspondiente
uint8_t tRele2Expan[] = {  //El indice es el numero de rele (1-48)
   NORELE,  //El indice cero no se usa
   1,0,3,2,5,4,7,6,14,15,12,13,10,11,8,9,   //Expansor 1
   1,0,3,2,5,4,7,6,14,15,12,13,10,11,8,9,   //Expansor 2
   1,0,3,2,5,4,7,6,14,15,12,13,10,11,8,9   //Expansor 3
};

//Esta tabla me relaciona la salida con el pin del expansor correspondiente
uint8_t tSalida2Expan[] = {  //El indice es el numero de salida
   0,  //El indice cero no se usa
   8,9,10,11,12,13,14,15   //Expansor 0 InOut
};


// **********  SENSORES Y SUS ENTRADAS ANALOGICAS Y DIGITALES   ***************

//Entradas digitales. Necesito saber el pin donde estan conectadas
#define MT0_ALTEMP_PIN 43  //PIN_A3=130643 -> 130600+43=130643
#define MT0_ALPRES_PIN 44  //PIN_A4
#define MT0_ALALTER_PIN 45 //PIN_A5
#define MT1_ALTEMP_PIN 57  //PIN_C1
#define MT1_ALPRES_PIN 58  //PIN_C2
#define MT1_ALALTER_PIN 59 //PIN_C3
#define ALACHIQUE1_PIN 69  //PIN_D5
#define ALACHIQUE2_PIN 70  //PIN_D6
#define ALACHIQUE3_PIN 71  //PIN_D7

//Aqui defino el nombre y el numero de sensor digital (secuencial, correlativo, sin huecos).
//Seran el idx de la matriz sensorDigi[]
//NX_FINDPAGE -> No es un sensor pero se inluye aqui para que se ejecute repetidamente
//ALARMA_ULTIMO -> se incluye solo para saber dinamicamente cual es el valor del ultimo sensor digital
enum
{
   NX_FINDPAGE,
   MT0_ALARMA_TEMP,
   MT0_ALARMA_PRES,
   MT0_ALARMA_ALTER,
   MT1_ALARMA_TEMP,
   MT1_ALARMA_PRES,
   MT1_ALARMA_ALTER,
   ALARMA_ACHIQUE1,
   ALARMA_ACHIQUE2,
   ALARMA_ACHIQUE3,
   ALARMA_ULTIMO
};

typedef struct
{
   int pin; //pin donde esta conectado
   int obj; //numero de objeto
} SENSOR_DIGI_STRUCT;
//En esta tabla asocio el numero de sensor digital con el pin donde esta conectado y con el objeto de la Nextion con el que interactuara
//sensor num
SENSOR_DIGI_STRUCT SensorDigi[] = {0, 0, MT0_ALTEMP_PIN, objpAlTemp0, MT0_ALPRES_PIN, objpAlPres0, MT0_ALALTER_PIN, objpAlAlter0,
                                   MT1_ALTEMP_PIN, objpAlTemp1, MT1_ALPRES_PIN, objpAlPres1, MT1_ALALTER_PIN, objpAlAlter1,
                                   ALACHIQUE1_PIN, objbAchique1, ALACHIQUE2_PIN, objbAchique2, ALACHIQUE3_PIN, objbAchique3}; //Quitar achiques ???

//Conversor analogico digital DAC ADS1115   ------------------------------------------

//A0
#define ADS1115_ADDR_0 0x90 //ADS1115  10010 00 0  -> 0x90 en modo write (b0=0)
#define ADS1115_ADDR_1 0x92 //ADS1115  10010 01 0  -> 0x92 en modo write (b0=0)
#define ADS1115_ADDR_2 0x94 //ADS1115  10010 10 0  -> 0x94 en modo write (b0=0)
#define ADS1115_ADDR_3 0x96 //ADS1115  10010 11 0  -> 0x96 en modo write (b0=0)
//Aqui asocio cada modulo con su direccion i2c
int ADS1115_addr[] = {ADS1115_ADDR_0, ADS1115_ADDR_1, ADS1115_ADDR_2, ADS1115_ADDR_3};

#define PRU_DIALES_CANAL 25 //Pruebas quitar despues

#define ADS1115_CONVER_REG 0x00
#define ADS1115_CONFIG_REG 0x01

#define ADC_MUX_SINGLE_0 4 * 4096 // Single-ended AIN0    0 100 0000 0000 0000    100(4)->AIN0
#define ADC_MUX_SINGLE_1 5 * 4096 // Single-ended AIN0    0 101 0000 0000 0000    101(5)->AIN1
#define ADC_MUX_SINGLE_2 6 * 4096 // Single-ended AIN2    0 110 0000 0000 0000    110(6)->AIN2
#define ADC_MUX_SINGLE_3 7 * 4096 // Single-ended AIN3    0 111 0000 0000 0000    111(7)->AIN3

int ADS1115_wConfig = 0x4183; //Config con el bit 15=0 -> No start conversion

//Aqui defino el nombre y el numero de sensor analogico (secuencial, correlativo, sin huecos).
//Seran el idx de la matriz sensor_canal[]
//SENSOR_ULTIMO -> se incluye solo para saber dinamicamente cual es el valor del ultimo sensor
enum
{
   MT0_SENSOR_TEMP,
   MT1_SENSOR_TEMP,
   MT0_SENSOR_PRES,
   MT1_SENSOR_PRES,
   MT0_SENSOR_ALTER,
   MT1_SENSOR_ALTER,
   MT_SENSOR_FUEL,
   MT_SENSOR_BAT1,
   MT_SENSOR_BAT2,
   MT_SENSOR_BAT3,
   SENSOR_ULTIMO
};

//Aqui relaciono cada sensor con su canal (Se puede cambiar en la Nexion)
#define NOCANAL 0xFF
//sensor
int sensor_canal[16] = {0, 1, 2, 3,
                        NOCANAL, NOCANAL, NOCANAL, NOCANAL,
                        NOCANAL, NOCANAL, NOCANAL, NOCANAL,
                        NOCANAL, NOCANAL, NOCANAL, NOCANAL}; // 16 canales disponibles, configurables en la Nextion

//Aqui relaciono el canal con su modulo y pin. Tabla fija, no se puede modificar
typedef struct
{
   int modulo; //Modulo donde esta conectado 0-3
   int pin;    //Pin del adc
} ADS1115_CANAL_STRUCT;
//canal                              //esta tabla me relaciona el canal con el modulo y el pin
ADS1115_CANAL_STRUCT ADS1115_Canal[] = {0, 0,    0,    1,     0, 2, 0, 3,
                                        1, 0, 1, 1, 1, 2, 1, 3,
                                        2, 0, 2, 1, 2, 2, 2, 3,
                                        3, 0, 3, 1, 3, 2, 3, 3};

/*
Abreviaturas Objetos que envian o reciben datos en la Nextion

Nextion  yo
-------  --  
         P   Pagina
t        t   Text
n        n   Number
x        f   Float
va       v   Variable
tm       T   Timer
b        b   Button
bt       d   Dual-state button
sw       s   Switch
j        j   Progress bar
m        h   Hostspot
z        a   Gauge / Aguja
s        w   Waveform
h        l   Slider
c        c   Check box
r        r   Check Radio
qr       q   QR code
cb       C   Combo Box
g        g   Scrolling text
select   e   Text Select
slt      E   Slider Text
p        p   Picture
*/

/* GUIA:
   -Ordenar la tabla NX_obj[] con el metodo de la burbuja, para asegurarnos que el indice esta correlativo
   -Recorrer la tabla NX_obj[] y si la pos de eeprom > 0, leerla y actualizar la tabla
   -Al cambiar de pagina, se actualizaran todos los objetos actualizables y visibles de esa pagina
   -Si salta un timer actualizamos los objetos correspondientes de la pagina activa
   -Si se reciben datos por el puero serie (objNum, valor), se procesaran y se actualizara se objeto en la Nextion

*/

/*
La Nextion me devuelve objNum, al ordenarla hacemos que idx=objNum
NX_obj[objNum].xxx accedo a los datos:
objNum, tipo, objname, atrib_cod, val_ant, confir[0], confir[1], EEpos, estado, Page[3].page, Page[3].vis, act, aux1, aux2; 


Si es un objeto tipo 'C' (combo) entonces aux2 es el indice de la tabla Pulsador_Actuador[] lo cual me da acceso
directo al rele con ActuadorRele[auxIdx].rele

Si es un objeto tipo 'b' o 'd' (pulsador) entonces auxIdx es el indice de la tabla tPulsadorActuador[].actuador[3] que me da acceso
a los actuadores que manejara ese pulsador. Cada actuador contiene el idx de la tabla ActuadorRele[] lo cual me da acceso
directo al rele con ActuadorRele[Idx].rele
*/

/*
boton=objNum -> NX_obj[objNum].aux2 -> Pulsado_Actuador[aux2] 



*/
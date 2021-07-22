#include "main.h"

/*
texto normal
* comentario resaltado
! comentario de alerta
? interrogacion
* @param miparam parametros para este metodo
TODO: pendiente de hacer algo
*/

/* 
TODO: Pendiente de hacer
   - Añadir un boton en la Nextion para guardar una copia de la eerpom en la SD,
o hacerlo automatiamente al desconectar las baterias.

*/


//*******  INTERRUPCIONES  *******************************************************************
portMUX_TYPE synch0 = portMUX_INITIALIZER_UNLOCKED; //Necesario para usar la int externa
portMUX_TYPE synch1 = portMUX_INITIALIZER_UNLOCKED; //Necesario para usar la int externa

//Int externa 0
void IRAM_ATTR extInt0()
{
   //Nota: Explorar la funcion pulseIn(), devuelve los microsegundos que esta un pulso en alto o en bajo
   //Si sumamos el tiempoen bajo + el tiempo en alto, tenemos el periodo
   portENTER_CRITICAL(&synch0); //Necesario, primera linea
   gRPM_pulsos_cont[MT0]++;     //Motor Izquierdo
   //Monitor.println("Int0");
   portEXIT_CRITICAL(&synch0); //Necesario, ultima linea
}

//Int externa 1
void IRAM_ATTR extInt1()
{
   portENTER_CRITICAL(&synch1); //Necesario, primera linea
   gRPM_pulsos_cont[MT1]++;     //Motor Izquierdo
   //Monitor.println("Int1");
   portEXIT_CRITICAL(&synch1); //Necesario, ultima linea
}

hw_timer_t *pTimer0 = NULL;                            //declaro un puntero al timer0
portMUX_TYPE timer0Mux = portMUX_INITIALIZER_UNLOCKED; //necesario int timer

//Int Timer0
void IRAM_ATTR onTimer0()
{                                      //Entrara cada 50ms
   portENTER_CRITICAL_ISR(&timer0Mux); //Necesario, primera linea
   flag_MT_sensor_time = 1;
   //digitalWrite(LED1_PIN,!digitalRead(LED1_PIN));
   portEXIT_CRITICAL_ISR(&timer0Mux); //Necesario, ultima linea
}

//******************************  SETUP  ********************************
void setup()
{
   //Pines entrada
   pinMode(VBAT_PIN, INPUT);
   pinMode(RTC_SQW_PIN, INPUT);
   pinMode(INT0_PIN, INPUT_PULLUP);
   pinMode(INT1_PIN, INPUT_PULLUP);

   //Pines salida
   pinMode(SD_CS_PIN, OUTPUT);
   digitalWrite(SD_CS_PIN, HIGH); //Lo ponemos a OFF (logica inversa)
   //pinMode(MOSFET_5V_PIN, OUTPUT);
   //digitalWrite(MOSFET_5V_PIN, LOW); //Lo ponemos a OFF
   pinMode(MOSFET_12V_PIN, OUTPUT);
   digitalWrite(MOSFET_12V_PIN, LOW); //Lo ponemos a OFF
   pinMode(LED1_PIN, OUTPUT);
   digitalWrite(LED1_PIN, LOW); //Lo ponemos a OFF
   pinMode(BUZZER_5V_PIN, OUTPUT);
   digitalWrite(BUZZER_5V_PIN, LOW); //Lo ponemos a OFF
   pinMode(BUZZER_12V_PIN, OUTPUT);
   digitalWrite(BUZZER_12V_PIN, LOW); //Lo ponemos a OFF

   Monitor.begin(115200, SERIAL_8N1, RX0_PIN, TX0_PIN); //baudrate, protocol, Rx, Tx
   Panta0.begin( 115200,SERIAL_8N1,RX1_PIN,TX1_PIN);
   //Panta1.begin( 256000,SERIAL_8N1,RX2_PIN,TX2_PIN);

   //setCpuFrequencyMhz(240); //En Arduino 240MHz por defecto
   Monitor.printf("\r\n\nESP32: Xtal=%uMHz, CPU=%uMHz, Flash=%fMB", getXtalFrequencyMhz(),getCpuFrequencyMhz(),spi_flash_get_chip_size()/1000000.0);
   //SPI_FLASH_MMAP_DATA,    /**< map to data memory (Vaddr0), allows byte-aligned access, 4 MB total */
   //SPI_FLASH_MMAP_INST,    /**< map to instruction memory (Vaddr1-3), allows only 4-byte-aligned access, 11 MB total */
   Monitor.printf("\r\nLibre data=%lu, program=%lu",spi_flash_mmap_get_free_pages(SPI_FLASH_MMAP_DATA), spi_flash_mmap_get_free_pages(SPI_FLASH_MMAP_INST));
   Monitor.printf("\r\nProg: %s v%u", PROG_NAME, PROG_VER);
   Monitor.printf("\r\nComp: %s %s\r\n", __DATE__, __TIME__);
   //SRAM?????
   //size_t flash = spi_flash_get_chip_size();

   //Iniciamos el puerto I2C0, y mapeamos los pines y la frecuencia
   Wire.begin(SDA_PIN, SCL_PIN, 400000); // SDA, SCL, 400kHz frequency
   //I2C0.begin(); // SDA, SCL, 400kHz frequency

   scanner_I2C();

   //***IMPORTANTE: Ignoro el motivo, pero hay que declarar los expansores aqui, si los
   //*coloco mas abajo no funcionan
   pcf0_inOut.begin(); //Iniciamos un dispositivo PCF8575 (I2C)
   pcf1_reles.begin(); //Iniciamos un dispositivo PCF8575 (I2C)
   pcf2_reles.begin(); //Iniciamos un dispositivo PCF8575 (I2C)
   pcf3_reles.begin(); //Iniciamos un dispositivo PCF8575 (I2C)

   //Ahora tenemos que configurar los expansores
   pcf0InOut_config();
   pcf1Reles_config();
   pcf2Reles_config();
   pcf3Reles_config();

   //Iniciamos el RTC
   uint8_t rtcStatus = DS3232_init();
   showDateTime();

   //Iniciamos SPI
   miSPI.begin(SD_SCK_PIN, SD_DO_PIN, SD_DI_PIN, SD_CS_PIN);
   //miSPI.transfer();

   //Iniciamos SD
   SD_Init();  
   
   ordena_tabla_NX_obj(); //Ordena la tabla en memoria
   //ordenar otras tablas ????????????
   Monitor.printf("\r\nNX_obj[] regs=0-%u", sizeof(NX_obj) / sizeof(NX_OBJ_STRUCT) - 1);
   //Comprueba la integridad de la eeprom y carga los valores en la tabla NX_obj
   uint8_t EEpromStatus = EEprom_Init(); 

   attachInterrupt(INT0_PIN, extInt0, RISING);
   attachInterrupt(INT1_PIN, extInt1, RISING);

   //En el ESP32 cada tick corresponde a 1Hz (en el PIC un tick son cada 4Hz)
   //Si la frecuencia del cristal es 80MHz. => 80000000ticks/seg
   pTimer0 = timerBegin(TMR0, TMR0_PREESCALER, true); //Si usamos un preescaler de 80. 80000000/80=1000000ticks/seg.
                                                      //osea que 1tick=0,000001seg=0,001ms=1us
   timerAttachInterrupt(pTimer0, &onTimer0, INT_RISING_EDGE);
   timerAlarmWrite(pTimer0, TMR0_50ms, ALARM_AUTORELOAD); //Entrara en la int onTimer0 cada 50ms
   timerAlarmEnable(pTimer0);

   Monitor.printf("\r\n\nComenzamos...\r\n");

   salida_set(1, 1); //Quitar despues

   previus_millis = current_millis = millis();
   time5seg = 0;
}

void loop()
{
   static bool i = 0;
   current_millis = millis();
   if(current_millis - previus_millis >=1000){  //Cada segundo
      previus_millis = current_millis;
      time5seg++;             //cuenta segundos
      if(time5seg==5){        //Cada 5 segundos
         time5seg = 0;
         if (!sd.status)               //Si SD no iniciada
            SD_Init();                 //Intenta iniciar la SD
      }
      digitalWrite(LED1_PIN, !digitalRead(LED1_PIN)); // Cada segundo Toggle led1
      
      //while(1){
         Panta0.print("Motor0.bClaxon.val=1");
         Panta0.write(0xFF);
         Panta0.write(0xFF);
         Panta0.write(0xFF);
         delay(1000);
         Panta0.print("Motor0.bClaxon.val=0");
         Panta0.write(0xFF);
         Panta0.write(0xFF);
         Panta0.write(0xFF);
         delay(1000);
         Monitor.printf("bClaxon.val=1");
         
      //}
   }

   if (!flag_bateria_on && digitalRead(VBAT_PIN))
   { //Si el usuario ha conectado las baterias
      flag_bateria_on = 1;
      Monitor.printf("\r\nVBat=ON");
      digitalWrite(MOSFET_12V_PIN, HIGH); //Activamos salida 12V

      //Configuramos los expansores de los reles
      pcf1Reles_config();
      pcf2Reles_config();
      pcf3Reles_config();

      //rele_set(2, 1);   //Quitar
     
     
   }
   else if (flag_bateria_on && !digitalRead(VBAT_PIN))
   {
      //rele_set(2, 0);   //Quitar despues
      digitalWrite(MOSFET_12V_PIN, LOW); //Desactivamos salida 12V
      flag_bateria_on = 0;
      Monitor.printf("\r\nVBat=OFF");
   }

      

   //Esta seccion gestionara los bytes recibidos en el puerto serie provenientes de las 2 pantallas nextion
   if (Panta0.available() > 0) //Si he recibido un comando por el puerto serie de PANTA0
   {
      readPanta0();
   }

   if (Panta1.available() > 0) //Si he recibido un comando por el puerto serie de PANTA1
   {
   }
}

//FUNCIONES -----------------------------------------------------------------

void scanner_I2C(void)
{
   byte status, address;
   int nDevices;

   Monitor.printf("\r\nI2C Scanning...\r\n");

   nDevices = 0;
   for (address = 1; address < 127; address++)
   {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      status = Wire.endTransmission();

      if (status == 0) //Si el dispositivo responde ACK OK
      {
         Monitor.print("Dispositivo I2C en 0x");
         if (address < 16)
            Monitor.print("0");
         Monitor.print(address, HEX);
         Monitor.print(" ->");
         switch (address)
         {
         case EXPANSOR0:
            Monitor.println(" Expansor 0, InOut");
            break;
         case EXPANSOR1:
            Monitor.println(" Expansor 1, Reles 1");
            break;
         case EXPANSOR2:
            Monitor.println(" Expansor 2, Reles 2");
            break;
         case EXPANSOR3:
            Monitor.println(" Expansor 3, Reles 3");
            break;
         case EEPROM_ADDR:
            Monitor.println(" Ext. EEprom");
            break;
         case DS3231_ADDRESS:
            Monitor.println(" RTC DS3231");
            break;
         }
         nDevices++;
      }
      else if (status == 4)
      {
         Monitor.print("Error dispositivo I2C en 0x");
         if (address < 16)
            Monitor.print("0");
         Monitor.println(address, HEX);
      }
   }
   if (nDevices == 0)
      Monitor.println("No hay dispositivos I2C\n");
}

//Configuramos expansor 0 InOut
void pcf0InOut_config(void)
{
   for (int n = 0; n < 16; n++)
   {
      if (n < 8) //El expansor0 tiene 8 pines de entrada y 8 de salida
         pcf0_inOut.pinMode(n, INPUT); //pines 8-15 como entrada
      else{
         pcf0_inOut.pinMode(n, OUTPUT);
         pcf0_inOut.digitalWrite(n, HIGH); //Pongo en Off los pines 0-7, necesario
      }
   }
}

//Configuramos expansor 1 Reles
void pcf1Reles_config(void)
{
   for (int n = 0; n < 16; n++)
   {
      pcf1_reles.pinMode(n, OUTPUT);
      pcf1_reles.digitalWrite(n, HIGH); //Pongo en Off todos los pines, necesario
   }
}

//Configuramos expansor 2 Reles
void pcf2Reles_config(void)
{
   for (int n = 0; n < 16; n++)
   {
      pcf2_reles.pinMode(n, OUTPUT);
      pcf2_reles.digitalWrite(n, HIGH); //Pongo en Off todos los pines, necesario
   }
}

//Configuramos expansor 3 Reles
void pcf3Reles_config(void)
{
   for (int n = 0; n < 16; n++)
   {
      pcf3_reles.pinMode(n, OUTPUT);
      pcf3_reles.digitalWrite(n, HIGH); //Pongo en Off todos los pines, necesario
   }
}

//Iniciamos el RTC
uint8_t DS3232_init(void)
{
   Monitor.printf("\r\nIniciando RTC...");
   uint8_t status = 0;
   if (!rtc.begin())
   {
      Monitor.printf("\r\nERROR: RTC no responde\r\n");
      Monitor.flush();
      status = 0; //No funciona
   }
   else
   {
      if (rtc.lostPower())
      {
         Monitor.printf("\r\nAVISO: RTC desajustado, ponemos fecha y hora por defecto");
         // When time needs to be set on a new device, or after a power loss, the
         // following line sets the RTC to the date & time this sketch was compiled
         rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
         // This line sets the RTC with an explicit date & time, for example to set
         // January 21, 2014 at 3am you would call:
         // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
         status = 1; //Funcionando, pero la hora no esta ajustada
      }
      else
      {
         status = 2; //En hora
         Monitor.printf(" OK\r\n");
      }
      rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
      //showDateTime();
      return status;
   }
}

//Comprueba la eeprom y carga los parametros en la tabla NX_obj
uint8_t EEprom_Init(void)
{
   Monitor.printf("\r\n\nIniciando EEPROM...");
   uint8_t status = 0;
   uint8_t byteCtrl = readEEprom(EEPROM_ADDR, 0);  //Leemos byte Ctrl
   if (byteCtrl == 0xAB) //OK
   {
      Monitor.printf(" OK (0x%X)\r\n", byteCtrl);
      Monitor.printf("\r\nCargamos parametros desde la EEprom...");
      EE_leo_datos();      //Carga los parametros desde la eeprom a las variables en memoria
      status = 1;
   }
   else
   {
      EE_graba_datos_defecto(); //Que son los valores asignados en la tabla NX_obj[] en su declaracion
      byteCtrl = readEEprom(EEPROM_ADDR, 0);
      Monitor.printf("\r\nbyteCtrl=0x%X", byteCtrl);
      if (byteCtrl == 0xAB) //OK
      {
         Monitor.printf("\r\nEEPROM VACIA (0x%X)\r\n", byteCtrl);
         Monitor.printf("\r\n****  AVISO: Usando parametros por defecto  ****\r\n");
         status = 2;
      }
      else
      {
         Monitor.printf("\r\nERROR: EEPROM no accesible (0x%X)\r\n", byteCtrl);
         status = 0;
      }
   }
   return status;
}

void writeEEprom(byte deviceaddress, uint16_t eeaddress, byte data)
{
   Wire.beginTransmission(deviceaddress);
   Wire.write((byte)(eeaddress >> 8));   //writes the MSB
   Wire.write((byte)(eeaddress & 0xFF)); //writes the LSB
   Wire.write(data);
   Wire.endTransmission();
   delay(10); //Necesario
   //Monitor.printf("\r\nwriteEEprom(): pos=%d, dato=%u",eeaddress, data);
}

//defines the readEEPROM function
uint8_t readEEprom(byte deviceaddress, uint16_t eeaddress)
{
   byte rdata = 0xFF;
   Wire.beginTransmission(deviceaddress);
   Wire.write((byte)(eeaddress >> 8));   //writes the MSB
   Wire.write((byte)(eeaddress & 0xFF)); //writes the LSB
   Wire.endTransmission();
   //Wire.requestFrom(deviceaddress, 1, true);
   Wire.requestFrom((uint8_t)deviceaddress, (uint8_t)1);
   if (Wire.available())
   {
      rdata = Wire.read();
      //Monitor.printf("\r\nreadEEprom(): pos=%d, dato=%u",eeaddress, rdata);
   }
   else
      Serial.println("No byte");

   return rdata;
}

uint16_t EE_graba_datos_defecto(void)
{
   Monitor.printf("\r\nEE_graba_datos_defecto()");

   Monitor.printf("\r\nGrabo byte de control 0xAB");
   writeEEprom(EEPROM_ADDR, 0, 0xAB); //Grabo byte de control en la pos 0
   uint16_t cont = 0;
   //Recorro la struc NX_obj[] y busco todas la variables que se guardan en la eeprom
   for (byte idx = 0; idx < (sizeof(NX_obj) / sizeof(NX_OBJ_STRUCT)); idx++)
   {
      if (NX_obj[idx].EEpos != NOEEP) //Si encuentro una posicion de la eeprom valida 0=NOEEPROM
      {
         if (NX_obj[idx].tipo == TACTUA) //Si es tipo Actuador debo guardar el rele asociado
         {
            byte rele = NX_obj[idx].aux2; //En los tipoN C, en aux2 contiene directamente el numero de rele de ese objeto (combo)
            writeEEprom(EEPROM_ADDR, NX_obj[idx].EEpos, rele);
            //Monitor.printf("\r\ntipoN=C, EEpos=%d, rele=%u",NX_obj[idx].EEpos,rele);
         }
         else //Si es otro tipoN grabo el estado en la eeprom
         {
            writeEEprom(EEPROM_ADDR, NX_obj[idx].EEpos, NX_obj[idx].estado);
            //Monitor.printf("\r\ntipoN=%c, EEpos=%d, estado=%u",NX_obj[idx].tipoN,NX_obj[idx].EEpos,NX_obj[idx].estado);
         }
         cont++;
      }
   }
   return cont;
}

//Leo todos los datos de la eeprom y los cargo en la tablas correspondientes
//OJO:tengo tambien que actuar sobre los reles, segun su estado????
uint16_t EE_leo_datos(void)
{
   byte idx;
   byte rele;
   byte estado;
   uint16_t cont = 0;

   Monitor.printf("\r\nEE_leo_datos()...Ctrl=0x%X", readEEprom(EEPROM_ADDR, 0));

   //Recorro la struc NX_obj[] y busco todas la variables que se guardan en la eeprom
   for (idx = 0; idx < (sizeof(NX_obj) / sizeof(NX_OBJ_STRUCT)); idx++)
   {
      if (NX_obj[idx].EEpos != NOEEP) //Si encuentro una posicion de la eeprom valida 0=NOEEPROM
      {
         if (NX_obj[idx].tipo == TACTUA) //Si es tipo Actuador
         {
            rele = readEEprom(EEPROM_ADDR, NX_obj[idx].EEpos); //Leo el rele guardado en esa posicion
            NX_obj[idx].aux2 = rele;                           //Pongo el rele en el campo aux2 de la tabla NX_obj[]
            //fprintf(MONITOR,"\r\ntipoN=C, EEpos=%Lu, rele=%u -> NX_obj[%u].aux2=%u",NX_obj[idx].EEpos,rele,idx,NX_obj[idx].aux2);
         }
         else //Si es otro tipo leo el estado guardado en la eeprom
         {
            estado = readEEprom(EEPROM_ADDR, NX_obj[idx].EEpos); //Leo el estado guardado en esa posicion
            NX_obj[idx].estado = estado;
            //fprintf(MONITOR,"\r\ntipoN=C, EEpos=%Lu, estado=%u -> NX_obj[%u].estado=%u",NX_obj[idx].EEpos,rele,idx,NX_obj[idx].estado);
         }
         cont++;
      }
   }
   Monitor.printf("\r\nDatos leidos de la EEprom: %u", cont);
   return cont;
}

void SD_Init(void)
{
   Monitor.printf("\r\nIniciando SD...");
   //miSPI.transfer();
   //Iniciamos la tarjeta SD, asignandole el puerto SPI deseado
   if (!SD.begin(SD_CS_PIN, miSPI))
   {  //si la tarjeta no esta insertada dara error
      Monitor.printf("\r\nERROR: Iniciando SD");
      sd.status = 0;
      sd.tipo = 9;
      sd.size = 0;
   }
   else
   {
      Monitor.printf(" OK");
      sd.status = 1;
      SD_tipo();
   }
}

bool SD_insert(void){   //No funciona como esperaba
   Monitor.printf("\r\nSD_insert()...");
   byte tipo = 0;
   if(sd.status){  //Si SD iniciada
      tipo = SD.cardType();
      if(tipo > CARD_NONE){
         Monitor.printf("tipo=%u R:1",tipo);
         return 1;
      }
   }
   Monitor.printf("0");
   return 0;
}

void SD_tipo(void){     //Solo funciona si la tarjeta esta insertada
   Monitor.printf("\r\nSD Card tipo: ");
   sd.tipo = SD.cardType();
   switch(sd.tipo){
      case CARD_NONE: //0
         Monitor.println("AVISO: No hay SD");   //?????????????
         break;
      case CARD_MMC: //1
         Monitor.println("MMC");
         break;
      case CARD_SD: //2
         Monitor.println("SDSC");
         break;
      case CARD_SDHC: //3
         Monitor.println("SDHC");
         break;
      default:
         Monitor.println("UNKNOWN");
         sd.tipo = 9;
         break;
   }
   sd.size = SD.cardSize() / (1024 * 1024);   //En MB
   Monitor.printf("SD Card Size: %lluMB", sd.size);
   if (sd.size > 3600)
      Monitor.print(" (4GB)");

   Monitor.printf("\r\nTotal bytes: ");
   Monitor.print(SD.totalBytes()/(1024*1024));  //En MB  //Return the total bytes enabled on SD. Returns bytes.

   Monitor.printf("\r\nUsed bytes: ");    
   Monitor.print(SD.usedBytes()/1024);   //En MB   //Return the total used bytes enabled on SD.Returns bytes
}

void readPanta0()
{ //Leemos los datos provenientes del la UART1 (Panta0)
   static byte n = 0;
   //Monitor.printf("\r\n\nreadPanta0(): ");
   while (Panta0.available())
   {
      char car = Panta0.read(); //Lee un caracter
      //Monitor.printf("\r\ncar=0x%X, %u, c=%c",car,car,car);
      //Monitor.println(char(Panta0.read()));
      if (car == 0xFC)
         n = 0; //Si es el caracter inicio, inicio el contador

      gRX_trama[NX0][n] = car; //Asigno el caracter a la trama de la pantalla 0 (NX0)

      if (n == TRAMA_FIN && gRX_trama[NX0][TRAMA_FIN] == 0xFD) //Si ncar[3] es el caracter Final
      {
         //Monitor.printf("\r\ntrama=0x%X 0x%X 0x%X 0x%X",gRX_trama[NX0][0],gRX_trama[NX0][1],gRX_trama[NX0][2],gRX_trama[NX0][3]);
         //La estructura del comando es:  0xFC,objnum,valor,0xFD
         Monitor.printf("\r\nRX0->%X, 0x%X, %u, %X", gRX_trama[NX0][TRAMA_INI], gRX_trama[NX0][TRAMA_OBJNUM],
                        gRX_trama[NX0][TRAMA_VAL], gRX_trama[NX0][TRAMA_FIN]);
         //panta,              ObjNum,                        val
         NX_ObjRcvAccion(NX0, gRX_trama[NX0][TRAMA_OBJNUM], gRX_trama[NX0][TRAMA_VAL]);
         //Si es una pagina, debemos ocultar los objetos no visibles de esa pagina desde el PIC
      }
      n++; //Siguiente caracter a procesar
   }
}

//Realiza la accion pertinente cuando se recibe un comando de la Nextion
//Cambiar valor variable, actualizar obj en la/las pagina/s, setear reles, write_eeprom
void NX_ObjRcvAccion(int panta, int objNum, int val)
{
   byte aux;
   byte actuadorIdx;
   byte rele;

   Monitor.printf("\r\nNX_obj_accion():pant=%u,ObjN=%u,Val=%d", panta, objNum, val);
   switch (objNum)
   {
   case objPPage:       //Si es una pagina
      gNX_page[panta] = val; //Actualizo el numero de pagina actual de esa pantalla. intentar canbia la variable a obj_NX[]
      Monitor.printf(" gNX_page[%u]=%u", panta, gNX_page[panta]);
      //ahora tengo que actualizar todos los datos visibles de la pagina actual de ambas pantallas, que no se actualizen periodicamente
      NX_actualiza_datos_pagina(gNX_page[panta]);
      break;
   case objpStart0:     //Si boton start/stop motor Izquierdo
      NX_obj[objNum].estado = val; //Guardamos el estado actual del boton 1=Pulsado,  0=Soltado
      if (val)                     //Si boton pulsado
      {
         tiempo_boton_start[MT0] = millis(); //Inicio el tiempo
         //flag_start_permiso_soltar_boton[MT0]=1;    //Activo permiso
         NX_obj[objNum].confir[1] = 1; //Activamos confirmacion soltar boton
      }
      flag_time_start[MT0] = 1; //Activo secuencia temporizador boton start1
      break;
      case objpStart1:     //Si boton start/stop motor Derecho

         break;
   case objpAlTemp0: //Boton monitor alarma temp motor 0
      digitalWrite(BUZZER_5V_PIN, LOW); //Silenciamos alarma
      break;
   default:    //Cualquier otro objNum
      Monitor.printf(",tipo=%u", NX_obj[objNum].tipo);

      switch (NX_obj[objNum].tipo)
      {
      case TPULSA:         //Manejamos los pulsadores
         NX_obj[objNum].estado = val; //Guardamos el estado actual del boton

         //Cofirmo el boton, si fuera necesario
         if (val) //Si boton ON
         {
            //buzzer_asincrono(1,50,50);  //Sonido pulsacion tecla
            if (NX_obj[objNum].confir[0])       //Si confirmar boton ON == 1
               NX_confirmar_boton(objNum, val); //Enviamos objeto.val=1
         }
         else //Si boton OFF
         {
            //if(NX_obj[objNum].tipo=='d') buzzer(1,50,50);  //Sonido pulsacion tecla, tambien al ponerla en OFF
            if (NX_obj[objNum].confir[1])       //Si confirmar boton OFF == 1
               NX_confirmar_boton(objNum, val); //Enviamos objeto.val=0
         }

         if (objNum == objbResetPic) //Si boton reset pic
         {
            ESP.restart(); //Reset general
            //esp_cpu_reset((uint16_t)xPortGetCoreID()); //reset por nucleo
            delay(5000);
         }

         //Seteamos los reles
         aux = NX_obj[objNum].aux2;
         if (aux != NOAUX) //Si existe aux, contiene el indice de la tabla Pulsador_Actuador[][]
         {
            for (uint8_t n = 0; n < PULSA_RELES_MAX; n++) //Un pulsador puede activar varios reles
            {
               //Si hay un actuador valido, contiene el indice del objeto actuador de la tabla NX_obj[], donde esta el rele
               if (Pulsador_Actuador[aux][n] != NORELE)
               {
                  actuadorIdx = Pulsador_Actuador[aux][n];
                  rele = NX_obj[actuadorIdx].aux2; //El rele esta en aux2 del objeto actuador (combo) de la tabla NX_obj[]
                  rele_set(rele, val);             //Activamos/Desactivamos el rele segun el estado del boton
               }
            }
         }
         break;
      case TACTUA:      //Los tipo Actuador contienen los numeros de reles 
         if (NX_obj[objNum].confir[1])       //Si confirmar Combo Box OFF==1
            NX_confirmar_boton(objNum, val); //Enviamos objeto
         //buzzer_asincrono(1,50,50);  //Sonido pulsacion tecla

         NX_obj[objNum].aux2 = val; //Guardo el nuevo valor del rele en aux2 en la tabla NX_obj[]

         //Si tiene un valor de pos EEprom valido
         if (NX_obj[objNum].EEpos != NOEEP)
            writeEEprom(EEPROM_ADDR, NX_obj[objNum].EEpos, val);  //Guardo en la eeprom el nuevo valor
         break;
      }
   }
}

//Actualiza todos los datos visibles de la pagina indicada de ambas pantallas, que no se actualizen periodicamente
void NX_actualiza_datos_pagina(int page)
{
   //int aux; //,rele;
   Monitor.printf("\r\nNX_actualiza_datos_pagina(): %u", page);

   for (int idx = 0; idx < (sizeof(NX_obj) / sizeof(NX_OBJ_STRUCT)); idx++) //Recorro la struc NX_obj[]
   {
      //Monitor.printf("\r\nidx=%u,objNum=%u,name=%s",idx,NX_obj[idx].objNum,NX_obj[idx].objname);
      //Envio a TODAS las pantallas todos los objetos con al condicion actualizable, y que NO se
      //actualicen periodicamente
      if (NX_obj[idx].act) //Si el dato es actualizable
      {
         if (NX_obj[idx].tipoN == 'C') //Si es un combo, tenemos que actualizar el valor del combo en la Nextion
         {
            if (NX_obj[idx].aux2 != NOAUX)                                          //Si aux2 tiene un valor, es el numero de rele
                                                                                    //panta,        ObjNum,        atr,          val,    forzar
               NX_send_obj(NXALL, NX_obj[idx].objNum, ATRVAL, NX_obj[idx].aux2, 1); //Pongo en cada combo el valor del rele asignado
         }
         else if (NX_obj[idx].tipoN == 'd') //Si es un dual state boton, actualizamos su estado
         {
            //panta,        ObjNum,        atr,          val,      forzar=1 Necesario para que actualice el estado del boton al cambiar de pantalla
            NX_send_obj(NXALL, NX_obj[idx].objNum, ATRVAL, NX_obj[idx].estado, 1); //Pongo cada boton en su estado
         }
      }
   }
   //Monitor.printf("\r\nFIN NX_send_datos()");
}

void NX_confirmar_boton(int objIdx, int val)
{
   int time1, time2; //Quitar despues

   Monitor.printf("\r\nNX_confirmar_boton(),page=%u ObjIdx=%u, val=%u, vis=%u", gNX_page[NX0], objIdx, val, NX_obj[objIdx].page[gNX_page[NX0]].vis);

   //Tengo que confirmarlo en las 2 pantallas, porque si tengo el mismo boton visible en las 2 pantallas
   //y lo pulso en una, tengo que confirmarlo tambien en la otra

   //Recorro las paginas simultaneas posibles de ese objeto
   //for(bool n=0; n<PAGE_REP;n++)                                //Intentar  evitar este bucle
   {
      //Si el objeto existe en la pagina actual     y  esta marcado como visible
      //if(NX_obj[ObjIdx].page[n].page==gNX_page[NX0] && NX_obj[ObjIdx].page[n].vis)
      if (esVisible(objIdx, gNX_page[NX0])) //Comprueba si existe y esta visible en la pagina actual de la Panta0
      {
         if (NX_obj[objIdx].tipoN == 'D') //Si es un boton triestado  ?????????
         {
            int bco2;

            if (val == 1) //Si esta en el segundo estado, cambio el fondo a verde
            {
               //bco2=1024;           //Verde
               //bco2=50712;          //Gris
               Panta0.printf("%s.txt=\"%s\"\xFF\xFF\xFF", NX_obj[objIdx].objName, "AUTO");    //Cambio el texto
               Monitor.printf("\r\n->%s.txt=%s\xFF\xFF\xFF", NX_obj[objIdx].objName, "AUTO"); //Cambio el texto
            }
            else if (val == 2) //Si esta en el tercer estado, cambio el fondo a rojo
            {
               bco2 = 63488; //Rojo
               //bco2=1024;           //Verde
               Panta0.printf("%s.txt=\"%s\"\xFF\xFF\xFF", NX_obj[objIdx].objName, "ON"); //Cambio el texto
               Monitor.printf("->%s.txt=%s\xFF\xFF\xFF", NX_obj[objIdx].objName, "ON");  //Cambio el texto

               Panta0.printf("%s.bco2=%d\xFF\xFF\xFF", NX_obj[objIdx].objName, bco2); //Cambio el color del boton
               Monitor.printf("\r\n->%s.bco2=%d\xFF\xFF\xFF", NX_obj[objIdx].objName, bco2);
               val = 1; //Necesario para confirmar el estado del boton ON, mas abajo
            }
            else
            {
               //bco2=50712;          //Gris
               Panta0.printf("%s.txt=\"%s\"\xFF\xFF\xFF", NX_obj[objIdx].objName, "OFF"); //Cambio el texto
               Monitor.printf("->%s.txt=%s\xFF\xFF\xFF", NX_obj[objIdx].objName, "OFF");  //Cambio el texto
            }
         }
         //if(val)
         {
            //Panta0.printf("%s.val=%u\xFF\xFF\xFF",NX_obj[ObjIdx].objname, val);
            int atrib_cod = NX_obj[objIdx].atrib_cod;
            time1 = millis();
            Panta0.printf("%s.%s=%u\xFF\xFF\xFF", NX_obj[objIdx].objName, NX_obj_atrib_txt[atrib_cod], val);
            time2 = millis();
            //Monitor.printf("\r\  n%s.val=%u\xFF\xFF\xFF",NX_obj[ObjIdx].objname, val);
            Monitor.printf("\r\n->%s.%s=%u\xFF\xFF\xFF,t=%dms", NX_obj[objIdx].objName, NX_obj_atrib_txt[atrib_cod], val, time2 - time1);
         }
      }

      //if(NX_obj[objIdx].page[n].page==gNX_page[NX1] && NX_obj[objIdx].page[n].vis)   //Miro en NX_obj[] si el objeto existe en la pagina activa de la NX1
      if (esVisible(objIdx, gNX_page[NX1])) //Comprueba si existe y esta visible en la pagina actual de la Panta1
      {
      }
   }
}

void rele_set(uint8_t rele, bool estado){
   uint8_t pin=tRele2Expan[rele];  //Averiguo el pin correspondiente
   Monitor.printf("\r\nrele_set(): R=%2u, estado=%u, pin=%u", rele, estado,pin);
   //El expansor dependera del numero de rele
   if(rele<=16)
      pcf1_reles.digitalWrite(pin, !estado); //Logica inversa
   else if(rele<=32)
      pcf2_reles.digitalWrite(pin, !estado); //Logica inversa
   else
      pcf3_reles.digitalWrite(pin, !estado); //Logica inversa
}

void salida_set(uint8_t salida, bool estado){
   uint8_t pin=tSalida2Expan[salida];  //Averiguo el pin correspondiente
   Monitor.printf("\r\nsalida_set(): S=%2u, estado=%u, pin=%u", salida, estado,pin);
   pcf0_inOut.digitalWrite(pin, !estado); //Logica inversa
}


//Envia objNum (si esta visible) a una pagina de una pantalla (o a todas panta=NXALL=0xFE), solo cuando
//el valor sea diferente del valor anterior (para reducir trafico), a no ser que se fuerce el cambio
void NX_send_obj(int panta, int ObjNum, int atr, int val, bool forzar)
{
   //Parametros:
   //-panta: Pantalla donde se enviara el dato (o todas panta=NXALL=0xEF)
   //-ObjNum: El codigo del objeto a enviar (ver tabla NX_obj[])
   //-val: Valor o estado del objeto a enviar
   //-forzar: envia el dato aunque sea igual que el valor anterior

   int idx = ObjNum;
   int time1, time2; //Quitar despues

   //Monitor.printf("\r\nsend_obj_val():panta=%u, page=%u, obj=%u, idx=%u, val=%Lu, v_ant=%Lu",
   //panta,gNX_page[NX0],ObjNum,Idx,val,NX_obj[Idx].val_ant);

   if (val != NX_obj[idx].val_ant || forzar) //Si el valor digital es diferente del valor anterior o forzar
   {
      //Ahora lo tengo que enviar a la pantalla indicada o a ambas (NXALL)
      //Lo envio a la panta0
      if (panta == NX0 || panta == NXALL) //Si es a la pantalla 0 (o a ambas)
      {
         //Si la Pantalla NX0 tiene alguna pagina valida asignada y el objeto es visible en esa pagina
         if (gNX_page[NX0] < NXPAGENULL && esVisible(NX_obj[idx].objNum, gNX_page[NX0]))
         {
            time1 = millis();
            Panta0.printf("%s.%s=%d\xFF\xFF\xFF", NX_obj[idx].objName, NX_obj_atrib_txt[atr], val);
            time2 = millis();
            Monitor.printf("\r\n*-->%s.%s=%d, t=%d", NX_obj[idx].objName, NX_obj_atrib_txt[atr], val, time2 - time1);
         }
      }

      //Lo envio a la panta1
      if (panta == NX1 || panta == NXALL) //Si es a la pantalla 1 (o a ambas)
      {
      }

      NX_obj[idx].val_ant = val; //Actualizo el valor anterior
   }
   //else
   //Monitor.printf("\r\nIgual valor");
}

//Solo comprueba si el obj existe en esa pagina
bool existe(int objNum, int page)
{
   for (int n = 0; n < PAGE_REP; n++) //compruebo todas la paginas posible de ese objeto
   {
      if (NX_obj[objNum].page[n].page == page) //si el objeto existe en esa pagina
         return 1;
   }
   return 0;
}

//Comprueba si existe y ademas esta visible en esa pagina
bool esVisible(int objNum, int page)
{
   for (int n = 0; n < PAGE_REP; n++) //compruebo todas la paginas posible de ese objeto
   {
      if (NX_obj[objNum].page[n].page == page && NX_obj[objNum].page[n].vis) //si el objeto existe en esa pagina y esta visible
         return 1;
   }
   return 0;
}

int ordena_tabla_NX_obj(void)
{
   int n, idx, r;
   int regs = (sizeof(NX_obj) / sizeof(NX_OBJ_STRUCT)); //Si divido el tamaño total de la tabla / tamaño de la structura = numero de filas o registros
   NX_OBJ_STRUCT temp;
   int errores = 0;

   Monitor.printf("\r\n\nordena_tabla_NX_obj()...");

   //Recorro la tabla NX_obj[] tantas veces como total de registros - 1
   for (n = 1; n < regs; n++)
   {
      //Recorro la tabla NX_obj[] desde el reg 0 hasta regs-1, regs-2, regs-3 ... hasta regs=1
      for (idx = 0; idx < regs - n; idx++)
      {
         if (NX_obj[idx].objNum > NX_obj[idx + 1].objNum) //si objnum es mayor que el siguiente
         {
            temp.objNum = NX_obj[idx].objNum; //guardamos el primer registro en una variable temporal
            temp.tipoN = NX_obj[idx].tipoN;
            temp.tipo = NX_obj[idx].tipo;
            strcpy(temp.objName, NX_obj[idx].objName);
            temp.atrib_cod = NX_obj[idx].atrib_cod;
            temp.val_ant = NX_obj[idx].val_ant;
            temp.confir[0] = NX_obj[idx].confir[0];
            temp.confir[1] = NX_obj[idx].confir[1];
            temp.EEpos = NX_obj[idx].EEpos;
            temp.estado = NX_obj[idx].estado;
            for (r = 0; r < PAGE_REP; r++)
            {
               temp.page[r].page = NX_obj[idx].page[r].page;
               temp.page[r].vis = NX_obj[idx].page[r].vis;
            }
            temp.act = NX_obj[idx].act;
            temp.aux1 = NX_obj[idx].aux1;
            temp.aux2 = NX_obj[idx].aux2;

            NX_obj[idx].objNum = NX_obj[idx + 1].objNum; //copiamos el segundo registro en el primero
            NX_obj[idx].tipoN = NX_obj[idx + 1].tipoN;
            NX_obj[idx].tipo = NX_obj[idx + 1].tipo;
            strcpy(NX_obj[idx].objName, NX_obj[idx + 1].objName);
            NX_obj[idx].atrib_cod = NX_obj[idx + 1].atrib_cod;
            NX_obj[idx].val_ant = NX_obj[idx + 1].val_ant;
            NX_obj[idx].confir[0] = NX_obj[idx + 1].confir[0];
            NX_obj[idx].confir[1] = NX_obj[idx + 1].confir[1];
            NX_obj[idx].EEpos = NX_obj[idx + 1].EEpos;
            NX_obj[idx].estado = NX_obj[idx + 1].estado;
            for (r = 0; r < PAGE_REP; r++)
            {
               NX_obj[idx].page[r].page = NX_obj[idx + 1].page[r].page;
               NX_obj[idx].page[r].vis = NX_obj[idx + 1].page[r].vis;
            }
            NX_obj[idx].act = NX_obj[idx + 1].act;
            NX_obj[idx].aux1 = NX_obj[idx + 1].aux1;
            NX_obj[idx].aux2 = NX_obj[idx + 1].aux2;

            NX_obj[idx + 1].objNum = temp.objNum; //copiamos temp en el segundo registro
            NX_obj[idx + 1].tipoN = temp.tipoN;
            NX_obj[idx + 1].tipo = temp.tipo;
            strcpy(NX_obj[idx + 1].objName, temp.objName);
            NX_obj[idx + 1].atrib_cod = temp.atrib_cod;
            NX_obj[idx + 1].val_ant = temp.val_ant;
            NX_obj[idx + 1].confir[0] = temp.confir[0];
            NX_obj[idx + 1].confir[1] = temp.confir[1];
            NX_obj[idx + 1].EEpos = temp.EEpos;
            NX_obj[idx + 1].estado = temp.estado;
            for (r = 0; r < PAGE_REP; r++)
            {
               NX_obj[idx + 1].page[r].page = temp.page[r].page;
               NX_obj[idx + 1].page[r].vis = temp.page[r].vis;
            }
            NX_obj[idx + 1].act = temp.act;
            NX_obj[idx + 1].aux1 = temp.aux1;
            NX_obj[idx + 1].aux2 = temp.aux2;
         }
      }
   }
   //Recorro la struc NX_obj[]
   for (idx = 0; idx < (sizeof(NX_obj) / sizeof(NX_OBJ_STRUCT)); idx++)
   {
      //Monitor.printf("\r\nidx=%u, objNum=%u, name=%s, aux1=%u",idx,NX_obj[idx].objNum,NX_obj[idx].objName,NX_obj[idx].aux1);
      if (idx != NX_obj[idx].objNum)
      {
         errores++;
         Monitor.printf("\r\nerror %u -> idx=%u, objNum=%u", errores, idx, NX_obj[idx].objNum);
      }
   }

   if (errores)
   {
      Monitor.printf("\r\n** %u errores Indice NX_obj[] **", errores);
      return errores;
   }

   Monitor.printf(" OK");
   return errores;
}

void showDateTime(void)
{
   DateTime now = rtc.now();

   Monitor.println();
   Monitor.print(now.year(), DEC);
   Monitor.print('/');
   Monitor.print(now.month(), DEC);
   Monitor.print('/');
   Monitor.print(now.day(), DEC);
   Monitor.print(" (");
   Monitor.print(daysOfTheWeek[now.dayOfTheWeek()]);
   Monitor.print(") ");
   Monitor.print(now.hour(), DEC);
   Monitor.print(':');
   Monitor.print(now.minute(), DEC);
   Monitor.print(':');
   Monitor.print(now.second(), DEC);
   Monitor.println();
}

//*****  FUNCIONES SD CARD  *************
void printDirectory(File dir, int numTabs)
{
   while (true)
   {

      File entry = dir.openNextFile();
      if (!entry)
      {
         // no more files
         break;
      }
      for (uint8_t i = 0; i < numTabs; i++)
      {
         Serial.print('\t');
      }
      Serial.print(entry.name());
      if (entry.isDirectory())
      {
         Serial.println("/");
         printDirectory(entry, numTabs + 1);
      }
      else
      {
         // files have sizes, directories do not
         Serial.print("\t\t");
         Serial.println(entry.size(), DEC);
      }
      entry.close();
   }
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
   Serial.printf("Listing directory: %s\n", dirname);

   File root = fs.open(dirname);
   if (!root)
   {
      Serial.println("Failed to open directory");
      return;
   }
   if (!root.isDirectory())
   {
      Serial.println("Not a directory");
      return;
   }

   File file = root.openNextFile();
   while (file)
   {
      if (file.isDirectory())
      {
         Serial.print("  DIR : ");
         Serial.println(file.name());
         if (levels)
         {
            listDir(fs, file.name(), levels - 1);
         }
      }
      else
      {
         Serial.print("  FILE: ");
         Serial.print(file.name());
         Serial.print("  SIZE: ");
         Serial.println(file.size());
      }
      file = root.openNextFile();
   }
}

void createDir(fs::FS &fs, const char *path)
{
   Serial.printf("Creating Dir: %s\n", path);
   if (fs.mkdir(path))
   {
      Serial.println("Dir created");
   }
   else
   {
      Serial.println("mkdir failed");
   }
}

void removeDir(fs::FS &fs, const char *path)
{
   Serial.printf("Removing Dir: %s\n", path);
   if (fs.rmdir(path))
   {
      Serial.println("Dir removed");
   }
   else
   {
      Serial.println("rmdir failed");
   }
}

void readFile(fs::FS &fs, const char *path)
{
   Serial.printf("Reading file: %s\n", path);

   File file = fs.open(path);
   if (!file)
   {
      Serial.println("Failed to open file for reading");
      return;
   }

   Serial.print("Read from file: ");
   while (file.available())
   {
      Serial.write(file.read());
   }
   file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
   Serial.printf("Writing file: %s\n", path);

   File file = fs.open(path, FILE_WRITE);
   if (!file)
   {
      Serial.println("Failed to open file for writing");
      return;
   }
   if (file.print(message))
   {
      Serial.println("File written");
   }
   else
   {
      Serial.println("Write failed");
   }
   file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
   Serial.printf("Appending to file: %s\n", path);

   File file = fs.open(path, FILE_APPEND);
   if (!file)
   {
      Serial.println("Failed to open file for appending");
      return;
   }
   if (file.print(message))
   {
      Serial.println("Message appended");
   }
   else
   {
      Serial.println("Append failed");
   }
   file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
   Serial.printf("Renaming file %s to %s\n", path1, path2);
   if (fs.rename(path1, path2))
   {
      Serial.println("File renamed");
   }
   else
   {
      Serial.println("Rename failed");
   }
}

void deleteFile(fs::FS &fs, const char *path)
{
   Serial.printf("Deleting file: %s\n", path);
   if (fs.remove(path))
   {
      Serial.println("File deleted");
   }
   else
   {
      Serial.println("Delete failed");
   }
}

void testFileIO(fs::FS &fs, const char *path)
{
   File file = fs.open(path);
   static uint8_t buf[512];
   size_t len = 0;
   uint32_t start = millis();
   uint32_t end = start;
   Serial.println("Test de lectura...");
   if (file)
   {
      len = file.size();
      size_t flen = len;
      start = millis();
      while (len)
      {
         size_t toRead = len;
         if (toRead > 512)
         {
            toRead = 512;
         }
         file.read(buf, toRead);
         len -= toRead;
      }
      end = millis() - start;
      Serial.printf("%u bytes read for %u ms\n", flen, end);
      file.close();
   }
   else
   {
      Serial.println("Failed to open file for reading");
   }

   file = fs.open(path, FILE_WRITE);
   if (!file)
   {
      Serial.println("Failed to open file for writing");
      return;
   }

   size_t i;
   Serial.println("Test de Escritura...");
   start = millis();
   for (i = 0; i < 2048; i++)
   {
      file.write(buf, 512);
   }
   end = millis() - start;
   Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
   file.close();
}

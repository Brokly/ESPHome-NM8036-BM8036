#pragma once

#ifndef NMBM8036_H
#define NMBM8036_H


#include <Arduino.h>
#include "esphome.h"
#include <stdarg.h>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/output/binary_output.h"

namespace esphome {
namespace nmbm8036 {
    
using sensor::Sensor;    
using text_sensor::TextSensor;    
using binary_sensor::BinarySensor;  
using time::RealTimeClock;
using uart::UARTDevice;  
using uart::UARTComponent;

typedef void(*on_switch_status)(bool); // тип обратного вызова для свичей

class NMBM8036;

class mn8036_Switch : public switch_::Switch, public Component, public esphome::Parented<NMBM8036> {
 protected:
   void write_state(bool state) override { 
      this->publish_state(state); 
      this->state_callback_.call(state);
    }
 friend class NMBM8036;   
};

struct sDallas{                      // структура датчика nm8036_DALLAS
    Sensor*       sensor=nullptr;    // указатель на сенсор ESPHOME для публикации показаний температуры
    TextSensor*   text_sens=nullptr; // указатель на сенсор ESPHOME для публикации серийного номера DALLASa
    int16_t       val=-13000;        // показания сенсора x100
    bool          change=false;      // флаг изменения показаний, для последующей публикации
    bool          isOk=false;        // текущий статус датчика (true - доступен)
    bool          oldIsOk=false;     // старый статус датчика
    uint8_t       sn[8]={0,0,0,0,0,0,0,0};// серийный номер датчика
    bool          snCh=false;        // признак необходимости публикации серийного номера
    float getTemp(){ return (((float)val)/100);} // показания температуры хуман-френдли
};
    
struct sChannel{                     // структура состояния выхода nm8036
    BinarySensor*  sensor=nullptr;   // указатель на сенсор ESPHOME для публикации состояния выхода
    bool           state=false;      // текущее остояние выхода
    bool           change=false;     // флаг изменения показаний, для последующей публикации
    bool           targManStatus=false; //целевое состояние на экране ручного управления
    mn8036_Switch* switch_= nullptr;  // наш переключатель
};
 
struct sBattery{
    Sensor*       sensor=nullptr;  // показания напряжения батареи
    uint16_t      value=0;         // буфер значения
    bool          change=false;    // флаг изменения показаний
    float getVolt(){ return (((float)value * 5)/1024);} // показания напряжения батареи
};    

struct sADC{                       // структура показаний входа nm8036
    Sensor*       sensor=nullptr;  // показания каналов АDC
    uint16_t      value=0;         // буфер значения
    bool          change=false;    // флаг изменения показаний
};    
 
struct sVersion{
    TextSensor*    sensor=nullptr; // версия прошивки устройства
    char*          buff=nullptr;   // указатель на буфер версии
    bool           change=true;    // флаг изменения данных
};
 
struct struct_clock{  // структура данных времени которую передает NM8036
    // байт №1
    uint8_t seconds:4; //секунды (4 бита)
    uint8_t ten_seconds:3; // десятки секунд (3 бита)
    uint8_t ch:1; // всегда = 0 (1 бит)
    // байт №2
    uint8_t minutes:4; // минуты
    uint8_t ten_minutes:3; // десятки минут
    uint8_t reserved_0:1;
    // байт №3
    uint8_t hours:4; // часы
    uint8_t ten_hours:2;// десятки часа
    uint8_t AMPM_24_mode:1; // всегда =0
    uint8_t reserved_1:1; // зарезервирован
    // байт №4
    uint8_t day:3; // День недели (1-7)
    uint8_t reserved_2:5;
    // байт №5
    uint8_t date:4; // число (1-31)
    uint8_t ten_date:2; // число (десятки)
    uint8_t reserved_3:2;
    // байт №6
    uint8_t month:4; // месяц (1-12)
    uint8_t ten_month:1; // месяц(десятки)
    uint8_t reserved_4:3;
    // байт №7
    uint8_t year:4; // год от 0 до 99
    uint8_t ten_year:4; // десятки года
    // байт №8
    uint8_t RS:2; // всегда =0
    uint8_t reserved_5:2;
    uint8_t SQWE:1; // всегда =1
    uint8_t reserved_6:2;
    uint8_t OUT:1; // всегда =1
};

struct sRTC{
    struct_clock clock; // данные времени
    bool needSet=false; // флаг необходимости установить время в NM8036
    uint32_t readTs=0 ; // системное время когда показания часов были считаны из NM8036, оно же флаг инициализации, не инициализировано, если = 0
};

// типы ошибок
enum _errType:uint8_t { _OK=0,        // ошибок нет
                        NO_REPLY,     // нет ответа
                        ER_REPLY,     // неизвестный ответ 
                        ER_SIZE,      // неправильный размер ответа
                        ER_START ,    // значение отличное от остальных, для старта
                        ER_REBOOT,    // перезагруза по ошибке
                        ER_UART_INIT, // в ямле не прописан UART
                        ER_FREEZE     // внешний девайс висит
};

#define IN_BUFF_SIZE 512      // размер входного буфера
#define OUT_BUFF_SIZE 32      // размер выходного буфера
#define REPLY_TIMEOUT 1000     // mS максимальный таймаут ожидания начала ответа от NM8036 
#define IN_BYTE_TIMEOUT 100    // 60mS максимальный таймаут между ожиданием байтов при ответе от NM8036 
#define OUT_DELAY  2000       // mS пауза между запросами отправки команд к NM8036 в нормальном режиме
#define OUT_FAST_DELAY  100   // пауза между запросами отправки команд к NM8036 в режиме ручного управления
#define SLOW_SERIAL_DELAY 5   // 5 mS задержка между байтами при медленной отправке
#define REBOOT_ERROR_COUNT  50 // счетчик ошибок потери связи, до перезагрузки устройства
#define MIN_PUBLISH_PERIOD 1000  // минимально возможный период публикации
#define TIMEOUT_FREEZE 30000 // таймаут зависания устройства

class NMBM8036 : public Sensor, public RealTimeClock {
 private:
    const char *const TAG = "nmbm8036";
    sDallas  T[32]; // массив сенсоров температуры
    sChannel O[12]; // массив выходов
    sBattery   Bat; // батарея
    sVersion  Vers; // версия ПО устройства
    sADC    Adc[4]; // показания ADC
    sRTC    my_rtc; // часы устройства
    TextSensor* displ1=nullptr; // строки дисплея
    TextSensor* displ2=nullptr;
    char dispStr1[17]={0}; //raw данные первой строки дисплея
    char dispStr2[17]={0}; //raw данные второй строки дисплея
    char* currLine=dispStr1; //активная строка меню, та у которой значек >
    
    // для эмуляции нажатия кнопок клавиатуры
    uint8_t keyBuff[16]={0}; // буфер кнопок
    uint8_t keyBuffIn=0; // указатель на вход для очереди отправки кнопку
    uint8_t keyBuffOut=0; // указатель на отправленную кнопку
    bool key_ok=true; // все кнопки отправлены
    #define KEY_ENTER 0 //типы возможных кнопок
    #define KEY_ARR_UP 1
    #define KEY_ARR_LF 2
    #define KEY_ARR_RT 3
    #define KEY_ARR_DN 4
    #define KEY_MENU 5
    #define KEY_PWR 6
    #define KEY_NO 0xFF
    #define KEY_YES 0xFE
    #define KEY_FAST 0x80    
    uint8_t targetManualOut=0; //Целевая строка для установки выхода
    #define MENU_UNDEF     0
    #define MENU_FIRST     1
    #define MENU_MANUAL    2
    //----------------------
    #define POS_CLOCK_SET  11
    #define POS_PROGR_SET  12
    #define POS_FIND_MEAS  13
    #define POS_PARAMS     14
    #define POS_ADC        15
    #define POS_OUT        16
    #define POS_MANUAL     17
    #define POS_LIGHT      18
    #define POS_CONTRAST   19
    #define POS_SOUND      20
    #define POS_SCREEN     21
    #define POS_VERSION    22
    //-----------------------
    #define POS_MANUAL_1    30
    #define POS_MANUAL_12   41
    uint8_t targetMenu=MENU_UNDEF; // по умолчанию рабочий экран
    mn8036_Switch* inter_switch=nullptr; // свитч для переключения в режим ручного управления
    uint8_t swBuff[16]={0}; // буфер изменения выходов
    uint8_t swBuffIn=0;     // указатель на вход для постановки в очередь
    uint8_t swBuffOut=0;    // указатель на точку обслуживания
    // для контроля и информировании о состоянии соединения
    UARTComponent *my_serial{nullptr};// указатель на UART для подключения к устройству
    TextSensor *error_string {nullptr}; // состояние канала связи RS232 текстовая рашифровка
    uint8_t error_count=0; // счетчик последовательных ошибок, для перезагрузки
    _errType uart_error=_OK; //состояние канала связи RS232  
    GPIOPin* signal_led{nullptr}; // нога с сигнальным диодом
    bool oldPinState = false; // текущее состояние светодиода статуса
    // переменные для организации трафика к устройсву
    uint8_t send_counter=0; // счетчик маршрутизатор отправки команд устройству
    uint8_t* comm=nullptr; //последовательность опроса, команды, ПОСЛЕ УДАЧНОГО ЧТЕНИЯ ВЕРСИИ 'V' заменяется на 'b' - чтение батарейки, см. ответ на 'V'
    uint8_t comm_counter=0; // размер буфера команд
    uint8_t posS=0; // позиция команды чтения экрана, для ускорения во время работы с клавой
    // V b - версия и батарейка всегда на первом месте
    #define P_DISP    0x0001 // будем читать дисплй d + S часто
    #define P_T_STAT  0x0002 // будем читать статусы датчиков v
    #define P_T_VAL   0x0004 // будем читать показания датчиков t
    #define P_T_SER   0x0008 // будем читать серийные номера датчиков D
    #define P_INPUT   0x0010 // будем читать состояния аналоговых входов s
    #define P_OUTPUT  0x0020 // будем читать состояния выходов l
    #define P_TIME    0x0040 // чтение часов устройства с 
    uint16_t protoFlg=P_TIME; // буфер флагов протокола для построения, время читаем всегда
    uint32_t out_last_time=0; // время отправки последнего байта к nm8036
    uint16_t out_size=0; //конец данных в буфере отправки
    uint16_t out_sended=0; //указатель на текущее место отправки данных
    uint32_t out_timeout=0; // задержка между отправляемыми байтами
    uint8_t  out_buff[OUT_BUFF_SIZE+1]={0}; // буфер отправки
    // переменные контроля и приема данных
    uint16_t in_size=0; //количество полученных байт
    uint8_t in_data[IN_BUFF_SIZE+1]={0}; //буфер получения данных
    uint32_t in_last_time=0; // время получения последнего байта от nm8036
    uint32_t in_timeout=REPLY_TIMEOUT; // опорное значение задержки приема
    uint32_t freeze_controll_timer=0; // таймер контроля зависания внешнего устройства
    uint32_t out_delay=OUT_DELAY; // параметр скорости опроса
    // вывод отладочной информации в лог
    // 
    // dbgLevel - уровень сообщения, определен в ESPHome. За счет его использования можно из ESPHome управлять полнотой сведений в логе.
    // msg - сообщение, выводимое в лог
    // line - строка, на которой произошел вызов (удобно при отладке)
    //
    // Своровал, спасибо GrKoR :)
    void _debugMsg(const String &msg, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0, ... ){
        if (dbgLevel < ESPHOME_LOG_LEVEL_NONE) dbgLevel = ESPHOME_LOG_LEVEL_NONE;
        if (dbgLevel > ESPHOME_LOG_LEVEL_VERY_VERBOSE) dbgLevel = ESPHOME_LOG_LEVEL_VERY_VERBOSE;
        if (line == 0) line = __LINE__; // если строка не передана, берем текущую строку
        va_list vl;
        va_start(vl, line);
        esp_log_vprintf_(dbgLevel, TAG, line, msg.c_str(), vl);
        va_end(vl);
    }
    
    // выводим данные пакета в лог для отладки
    // 
    // dbgLevel - уровень сообщения, определен в ESPHome. За счет его использования можно из ESPHome управлять полнотой сведений в логе.
    // packet - указатель на пакет для вывода;
    //          если указатель на crc равен nullptr или первый байт в буфере не AC_PACKET_START_BYTE, то считаем, что передан битый пакет
    //          или не пакет вовсе. Для такого выводим только массив байт.
    //          Для нормального пакета данные выводятся с форматированием. 
    // line - строка, на которой произошел вызов (удобно при отладке)
    //
    void _debugPrintPacket(uint8_t* data, uint16_t size, bool in, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0){
        String st = "";
        char textBuf[11];
        // заполняем время получения пакета
        memset(textBuf, 0, 11);
        sprintf(textBuf, "%010u:", millis());
        st = st + textBuf;
        // формируем преамбулы
        if (in) {
            st += "[<=] ";      // признак входящего пакета
        } else {
            st += "[=>] ";      // признак исходящего пакета
        } 
        for (uint16_t i=0; i<size; i++){
            memset(textBuf, 0, 11);
            sprintf(textBuf, "%02X", data[i]);
            st += textBuf;
            st +=' ';
        }
        if (line == 0) line = __LINE__;
        _debugMsg(st, dbgLevel, line);
    }

    // текстовая расшифровка ошибок
    char* getStrError(_errType lastError){
        static char out[40]={0};
        char* rep=out;
        if(lastError==_OK){
            strcpy(rep,"OK");
        } else if (lastError==NO_REPLY){
            strcpy(rep,"Reply timeout");
        } else if (lastError==ER_REPLY){
            strcpy(rep,"Reply incorrect");
        } else if (lastError==ER_SIZE){
            strcpy(rep,"Size reply incorrect");
        } else if (lastError==ER_REBOOT){
            strcpy(rep,"Reboot, serial communication error");
        } else if (lastError==ER_UART_INIT){
            strcpy(rep,"UART undefined, check config");
        } else if (lastError==ER_FREEZE){
            strcpy(rep,"NMBM8036 freeze");
        } else {
            strcpy(rep,"Unexpected");
        }
        return out;
    }    
    
    // управление ногой светодиода
    void set_pin_state(bool state){
        if(oldPinState!=state){
            oldPinState=state;
            if(signal_led!=nullptr){
                signal_led->digital_write(state);
            }
        }           
    }

    // зарузка данных в буфер отправки
    bool sendSerialData(uint8_t* data, uint8_t size, uint32_t delay=0){
        out_timeout=delay; // между отправкой байтов должна быть такая задержка
        if(out_size+size>OUT_BUFF_SIZE){ // проверка на переаолнение буфера отправки
            _debugMsg(F("%010u: Out buffer overflow !"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());
            return false; // данные в буфер не заносим !
        }
        memcpy(&(out_buff[out_size]),data, size); //копируем данные в конец буфера
        out_size+=size;
        return true;
    }
    bool sendSerialData(uint8_t data, uint32_t delay=0){
        return sendSerialData(&data, 1, delay);
    }

    // построение буфера команд в зависимости от подключеных сенсоров
    void createComm(){
       comm_counter=3; //V+b
       uint16_t buff=protoFlg;
       bool disp=(buff & 1); // признак чтения дисплея
       buff>>=1;
       const char tbl[]="vtDslc";
       // считаем нужный размер буфера команд
       while(buff){
          if(buff & 1){
             comm_counter++; // для текущей команды
             if(disp){
                comm_counter+=2; // для опроса дисплея
             }
          }
          buff>>=1;
       }
       comm=(uint8_t*)malloc(comm_counter+1); //получили буфер
       if(comm==nullptr){
          comm_counter=0;
          ESP_LOGE(TAG, "In bilding command line memory allocation fatal error !"); 
       } else {
          uint8_t i=3;
          uint8_t j=0;
          buff=protoFlg>>1;
          comm[0]='V'; //V<=>b - всегда
          comm[1]='d'; //d+s - всегда, для контроля зависания
          comm[2]='S'; //V+b - всегда
          posS=2; // тут чтение экрана, для ускорения обработки
          while(buff){
              if(buff & 1){
                comm[i++]=tbl[j]; // для текущей команды
                if(disp){
                   comm[i++]='d'; // для опроса дисплея
                   comm[i++]='S'; // для опроса дисплея
                }
             }
             if(j<sizeof(tbl)){ 
                j++; // к сл. команде
             }
             buff>>=1;
          }
          comm[comm_counter]=0; // на всякий случай
          ESP_LOGD(TAG, "Bilding command line: ' %s ', size:%u", comm, comm_counter); 
       }
    }
    
    // получение кастомных символов из памяти LCD
    char* getCustomLCDSimb(uint8_t i, uint8_t* mem){
       static char ret[3]={0}; 
       ret[0]=' '; 
       ret[1]=0;       
       if(i<8){ // в служебной памяти всего 8 символов
          uint64_t* addr= (uint64_t*)(i*8+mem); // указатель на наш символ
          if     (*addr==0x001F111B1B131B1F) ret[0]='1';
          else if(*addr==0x00000E04040C0400) ret[0]='1';
          else if(*addr==0x001F1117111D111F) ret[0]='2';
          else if(*addr==0x00000E080E020E00) ret[0]='2';
          else if(*addr==0x001F111D111D111F) ret[0]='3';
          else if(*addr==0x00000E020E020E00) ret[0]='3';
          else if(*addr==0x001F1D1D1115151F) ret[0]='4';
          else if(*addr==0x000002020E0A0A00) ret[0]='4';
          else if(*addr==0x001F111D1117111F) ret[0]='5';
          else if(*addr==0x00000E020E080E00) ret[0]='5';
          else if(*addr==0x001F11151117111F) ret[0]='6';
          else if(*addr==0x001F1D1D1D1D111F) ret[0]='7';
          else if(*addr==0x0000020202020E00) ret[0]='7';
          else if(*addr==0x001F11151115111F) ret[0]='8';
          else if(*addr==0x00000E0A0E0A0E00) ret[0]='8';
          else if(*addr==0x001F111D1115111F) ret[0]='9';
          else if(*addr==0x00000E020E0A0E00) ret[0]='9';
          else if(*addr==0x001F11151515111F) ret[0]='0';
          else if(*addr==0x00000E0A0A0A0E00) ret[0]='0';
          else if(*addr==0x00080C1E1F1E0C08) ret[0]='>';
          else {
             ESP_LOGE("","Unexpected string line :%u 0x%08X%08X", i, *(((uint32_t*)addr+1)),*((uint32_t*)addr));
          }
       }
       return ret;
    }

    // декодирование строки дисплея 1602
    char* decodeLCDLine(uint8_t* in, uint8_t* mem){
       static uint8_t line[17]={0};
       // таблица перекодировки LCD
       const uint8_t lcd2ascii[256]={
            0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
            0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
             ' ', '!',0x22,0x23, '$', '%', '&',0x27, '(', ')', '*', '+', ',', '-', '.', '/',
            0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,
            0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,
            0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,
            0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,
            0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x20,0x20,0xBF,0x7E,0x20,
            0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
            0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
            0xC1,0xC3,0xA8,0xC6,0xDD,0xC8,0xC9,0xCB,0xCF,0xD3,0xD4,0xD7,0xD8,0xDA,0xDB,0xDD,
            0xDE,0xDF,0xE1,0xE2,0xE3,0xB8,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEF,0xF2,
            0xF7,0xF8,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF,0x8B,0x94,0x84,0xCA,0xB9,0x3F,0x3F,0x3F,
            0x83,0xBF,0x3F,0x3F,0xA2,0x78,0xA1,0x7C,0x3F,0x3F,0x3F,0x7B,0x7D,0x3F,0x3F,0x07,
            0xC4,0xD6,0xD9,0xE4,0xF4,0xF6,0xF9,0x92,0x93,0xB8,0xAA,0x3F,0x3F,0x3F,0x03,0x09,
            0x3F,0x3F,0x3F,0x3F,0x3F,0xB1,0x3F,0x3F,0x86,0x3F,0x3F,0x87,0x3F,0x05,0x90,0x06
        };
        for(uint8_t i=0;i<16;i++){
           if(in[i]<8){
              line[i]= getCustomLCDSimb(in[i], mem)[0];
           } else {
              line[i]=lcd2ascii[in[i]];   
           }
        }
        return (char*)line;
    }
    
    // конвертинг одного символа из СP1251 в UNICODE
    char* convRusUni(char c){
       static char ret[3]={0};
       if(c>239){ //240...255 р...я (p=D180....я=D18F )
          ret[0]=0xD1;
          ret[1]=c-112;
       } else if(c>191){ // 192... 239 A...п (А=D090....п=D08F)
          ret[0]=0xD0;
          ret[1]=c-48;
       } else if(c==168){ // Ё
          ret[0]=0xD0;
          ret[1]=0x81;
       } else if(c==184){ // ё
          ret[0]=0xD1;
          ret[1]=0x91;
       } else {
          ret[0]=c;
          ret[1]=0;
       }
       return ret;
    }
    
    // конвертинг строки из СP866 в UNICODE
    char* strUni(char* in){
       static char er=0;
       static char* out=nullptr;
       static size_t sizeBuff=0;
       size_t needSize=strlen(in)*2+1; // максимально возможный размер строки после конвертации
       if(sizeBuff<needSize){ // нужен буфер больше
          if(out!=nullptr){
             free(out);
          }
          out=(char*) malloc(needSize);
          sizeBuff=needSize;
       }
       if(out==nullptr){
          ESP_LOGE(TAG,"Fatal error alocation memory in string decoding procedure");
          sizeBuff=0;
          return &er;
       } 
       char* _out=out;
       while(*in){
          char* conv=convRusUni(*in);
          *_out=conv[0];
          _out++;
          if(conv[1]!=0){
             *_out=conv[1]; 
             _out++;                
          }
          in++;
       }
       *_out=0;
       return out;       
    }
    
    // для отладки
    void printKeyBuff(){
       String out="Key buff: ";
       uint8_t temp=keyBuffOut; // первый байт в кольцевом буфере
       while(keyBuffIn != temp){ // ищем нужный номер свитча
          out+=String(keyBuff[temp]);
          out+=",";
          temp=(temp+1)&0xF;
       }
       ESP_LOGE("DEBUG",out.c_str());
    }
    
    // постановка кода кнопки для отправки в буфер
    bool setKey(uint8_t key){
        uint8_t temp=(keyBuffIn+1) & 0xF;
        if(temp != keyBuffOut){
           keyBuff[keyBuffIn]=key;
           key_ok=false; //в буфере есть данные для отправки
           keyBuffIn=temp;
           ESP_LOGD("","Set key in buff for send: %u",key); 
           //printKeyBuff();           
           return true;
        }
        keyBuffIn=(keyBuffIn-1) & 0xF;
        return false; // нет места в буфере
    }

    // получение кода кнопки для отправки из буфера
    uint8_t getKey(){
        if(keyBuffIn != keyBuffOut){
           uint8_t temp=keyBuff[keyBuffOut];
           keyBuffOut=(keyBuffOut+1)&0xF;
           ESP_LOGD("","Get key from buff for send: %u",temp);            
           //printKeyBuff();           
           return temp;      
        }
        return 0x20; // в буфере ничего нет
    }

    // проверка наличия неотправленных кнопок в буфере отправки
    inline bool availableKey(){
       return (keyBuffIn != keyBuffOut);
    }

    // установка кнопки только при пустом буфере
    bool set_one_key(uint8_t val){   
       if(key_ok){ // кнопок в буфере нет
           //ESP_LOGD("","Set key for send: %u",val);            
           setKey(val);
           send_counter=posS; //ускорение чтение дисплея           
           out_delay=OUT_FAST_DELAY; // усорение обмена данными
           return true;
       }
       return false;
    }       

    // для отладки
    void printSwBuff(){
       String out="Sw buff: ";
       uint8_t temp=swBuffOut; // первый байт в кольцевом буфере
       while(swBuffIn != temp){ // ищем нужный номер свитча
          out+=String(swBuff[temp]);
          out+=",";
          temp=(temp+1)&0xF;
       }
       ESP_LOGE("DEBUG",out.c_str());
    }

    // очередь изменения номеров входов, требующая обслуживания
    // добавление номера входов 0...11, флаг состояния в старшем бите
    bool setSw(uint8_t num){
        //ESP_LOGD("","Set switch number %u to target, from buffer",num);            
        uint8_t temp=swBuffOut; // первый байт в кольцевом буфере
        while(swBuffIn != temp){ // ищем нужный номер свитча
           if(swBuff[temp] == num){ // нашли !
              return true;   
           }
           temp=(temp+1)&0xF;
        }
        temp=(swBuffIn+1) & 0xF;
        if(temp != swBuffOut){
            swBuff[swBuffIn]=num;
            swBuffIn=temp;
            return true;
        }
        return false; // нет места в буфере
    }

    // получение номера свича из буфера для обслуживания
    int8_t getSw(){
        if(swBuffIn != swBuffOut){
           uint8_t num=swBuff[swBuffOut];
           swBuffOut=(swBuffOut+1)&0xF;
           //ESP_LOGD("","Get switch number %u, for set to target",num);            
           return num;      
        }
        return -1; // в буфере ничего нет
    }

    // проверка наличия необслуженных свичей
    inline bool availableSw(){
       return (swBuffIn != swBuffOut);
    }

   // определение текущего состояния экрана
    void getScrPos(uint8_t* menu, uint8_t* active){
       uint64_t temp=0;
       currLine=dispStr1;
       if(dispStr1[0]==0x3E){ // активна верхняя строка
          *menu=MENU_FIRST; // первая глубина меню
          temp= *((uint64_t*)(dispStr1+2));// активная строка 1
       } else if(dispStr2[0]==0x3E){ // активна нижняя строка
          *menu=MENU_FIRST; // первая глубина меню
          temp= *((uint64_t*)(dispStr2+2));// активная строка 2
          currLine=dispStr2;
       } else { // вообще не та страница меню
          *menu=MENU_UNDEF; // не опознанная глубина меню
       }
       if(*menu==MENU_FIRST){ // опознаем положение на первой странице меню
          if(temp==0xF72061EA2DF263D3){ //Уcт-кa чacoв
             *active=POS_CLOCK_SET;
          } else if(temp==0xECEC6170E36F70CF){ //Пpoгpaммa yпp.
             *active=POS_PROGR_SET;
          } else if(temp==0x61E420EA63E86FCF){ //Пoиcк дaтчикoв
             *active=POS_FIND_MEAS;
          } else if(temp==0x70F265EC617061CF){ //Пapaмeтpы
             *active=POS_PARAMS;
          } else if(temp==0xEAE26FED61F263D3){ //Уcтaнoвки AЦП
             *active=POS_ADC;
          } else if(temp==0xFBE220ECE8E66550){ //Peжим выxoдoв
             *active=POS_OUT;
          } else if(temp==0x7920656FEDF77950){ //Pyчнoe yпpaвл.
             *active=POS_MANUAL;
          } else if(temp==0xEAF265E263E46FCF){ //Пoдcвeткa
             *active=POS_LIGHT;
          } else if(temp==0xF2636170F2ED6F4B){ //Koнтpacтнocть
             *active=POS_CONTRAST;
          } else if(temp==0x20202020EA79E2DD){ //Эвyк
             *active=POS_SOUND;
          } else if(temp==0xEAFD20ECE8E66550){ //Peжим экpaнa
             *active=POS_SCREEN;
          } else if(temp==0x3120FFE863706542){ //Bepcия 1.95
             *active=POS_VERSION;
          } else if(temp==0x202320E46F78FB42 || temp==0x312320E46F78FB42){ //Bыxoд #  НАХОДИМСЯ НА СТРАНИЦЕ РУЧНОГО УПРАВЛЕНИЯ
             *menu=MENU_MANUAL;
             uint8_t num=currLine[10]-'0'; // читаем номер ноги ручного управления
             if(currLine[9]>='0' && currLine[9]<='9'){
                 num+=10*(currLine[9]-'0');
             }                 
             *active=num+POS_MANUAL_1-1;
         }
       }
    }

    // идентифицированый вызов изменения статуса переключателя
    void getStat(bool st, uint8_t num){
       if(O[num].switch_!=nullptr && inter_switch!=nullptr &&  inter_switch->state){
          setSw(num); //ставим переключатель на обслуживание
          out_delay=OUT_FAST_DELAY; //пауза между запросами отправки команд к NM8036 в режиме анализа экрана
       }
    }

    // сброс всех свичй
    void clearSwitchs(){
       for(uint8_t i=0; i<sizeof(O)/sizeof(O[0]); i++){
          if(O[i].switch_!=nullptr && O[i].switch_->state){ // свич инициализирован и включен
             O[i].switch_->turn_off();
          }
       }    
       // сбросить буфер очереди обслуживания свитчей
       swBuffIn=0;     // указатель на вход для постановки в очередь
       swBuffOut=0;    // указатель на точку обслуживания
     }

 public:
    // подключение переключателей
    void set_switches(mn8036_Switch* switch_, uint8_t num) {  // хрень конечно....
       if(num<sizeof(O)/sizeof(O[0])){ 
          O[num].switch_ = switch_; 
          if     (num==0) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,0);});              
          else if(num==1) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,1);});              
          else if(num==2) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,2);});              
          else if(num==3) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,3);});              
          else if(num==4) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,4);});              
          else if(num==5) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,5);});              
          else if(num==6) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,6);});              
          else if(num==7) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,7);});              
          else if(num==8) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,8);});              
          else if(num==9) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,9);});              
          else if(num==10) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,10);});              
          else if(num==11) switch_->add_on_state_callback([this](bool st){if(!std::isnan(st)) getStat(st,11);});              
          protoFlg|=P_DISP;
       }
    }
    // подключение переключателя перехвата управления
    void set_hook_switches(mn8036_Switch* switch_){
       static uint8_t oldState=true;
       inter_switch=switch_;
       protoFlg|=P_DISP;
       switch_->add_on_state_callback([this](bool st){ // переключение свича захвата управления
            if(!std::isnan(st)){
               if(oldState!=st){ // свитч изменился, все ведомые свичи - отключить
                  oldState=st;  
                  clearSwitchs();                  
               }
               if(inter_switch!=nullptr){
                  inter_switch->publish_state(st); 
               }
               if(st){ // свитч активирован 
                  out_delay=OUT_FAST_DELAY; //пауза между запросами отправки команд к NM8036 в режиме анализа экрана
               } 
            }
       });
    }
    // подключение текстовых сенсовров строк дисплея
    void set_disp_str1(text_sensor::TextSensor* str_sensor) { displ1=str_sensor; protoFlg|=P_DISP;}
    void set_disp_str2(text_sensor::TextSensor* str_sensor) { displ2=str_sensor; protoFlg|=P_DISP;}
    // подключение температурных сенсоров
    void set_temperature_sensor(sensor::Sensor *temperature_sensor, uint8_t num) { if(num<sizeof(T)/sizeof(T[0])){ T[num].sensor = temperature_sensor; protoFlg|=P_T_VAL|P_T_STAT;}}
    // подключение сенсоров серийных номеров термосенсоров
    void set_dallas_sn_sensor(text_sensor::TextSensor *SN_sensor, uint8_t num) { if(num<sizeof(T)/sizeof(T[0])){ T[num].text_sens = SN_sensor; protoFlg|=P_T_SER|P_T_STAT;}}
    // подключение сенсоров выходов
    void set_output_sensor(binary_sensor::BinarySensor *out_sensor, uint8_t num) { if(num<sizeof(O)/sizeof(O[0])){ O[num].sensor = out_sensor; protoFlg|=P_OUTPUT;}}
    // подключение сенсоров выходов (ADC) 
    void set_adc_sensor(sensor::Sensor *adc, uint8_t num) { if(num<sizeof(Adc)/sizeof(Adc[0])){ Adc[num].sensor = adc; protoFlg|=P_INPUT;}}
    // батарейка
    void set_batt_sensor(sensor::Sensor *bat_sensor) { Bat.sensor = bat_sensor;}
    // версия
    void set_vers_sensor(text_sensor::TextSensor *vers_sensor) { Vers.sensor = vers_sensor;}
    // сенсор статуса обмена по RS232
    void set_err_sensor(text_sensor::TextSensor *err_sensor) { error_string = err_sensor;}
    // подключение последовательного интерфейса
    void initUart(UARTComponent *parent = nullptr){ my_serial=parent;}  
    // подключение сигнального диода
    void set_pin(GPIOPin  *pin = nullptr){ signal_led=pin; signal_led->setup();}  
    // приоритет потока      
    float get_setup_priority() const override { return esphome::setup_priority::LATE;}
    // вывод в дебаг текущей конфигурации компонента
    void dump_config() {
        ESP_LOGCONFIG(TAG, "NM/BN8036:");
        char textBuf[20];
        // перечисление свойств датчиков температуры
        for(uint8_t i=0; i<sizeof(T)/sizeof(T[0]); i++){
            if(T[i].sensor!=nullptr){
                sprintf(textBuf, "Dallas sensor %d", i); 
                LOG_SENSOR("", textBuf, this->T[i].sensor);
            }
            if(T[i].text_sens!=nullptr){
                sprintf(textBuf, "Dallas sensor serial %d", i); 
                LOG_TEXT_SENSOR("", textBuf, this->T[i].text_sens);
            }
        }
        // если есть переключатель управления
        if(inter_switch!=nullptr){
            LOG_SWITCH("", "Interceptor switch", inter_switch);
        }
        // перечисление выходов
        for(uint8_t i=0; i<sizeof(O)/sizeof(O[0]); i++){
            if(O[i].sensor!=nullptr){
                sprintf(textBuf, "Channel %d output ", i); 
                LOG_BINARY_SENSOR("", textBuf, this->O[i].sensor);
            }
            if(O[i].switch_!=nullptr){
                sprintf(textBuf, "Switch %d output ", i); 
                LOG_SWITCH("", textBuf, this->O[i].switch_);
            }
        }
        // перечисление ADC
        for(uint8_t i=0; i<sizeof(Adc)/sizeof(Adc[0]); i++){
            if(Adc[i].sensor!=nullptr){
                sprintf(textBuf, "ADC Channel %d ", i); 
                LOG_SENSOR("", textBuf, this->Adc[i].sensor);
            }
        }
        // часы устройства
        ESP_LOGCONFIG(TAG, "Inbound Real Time Clock");
        ESP_LOGCONFIG(TAG, "Timezone: '%s'", this->timezone_.c_str());
        // остальные данные
        LOG_SENSOR("", "RTC Battery Status", this->Bat.sensor);
        LOG_TEXT_SENSOR("", "Firmware Version", this->Vers.sensor);
        LOG_TEXT_SENSOR("", "Last Error", this->error_string);
        // uart
        ESP_LOGCONFIG(TAG, "UART Bus:");
        //LOG_PIN("  TX Pin: ", my_serial->tx_pin_);
        //LOG_PIN("  RX Pin: ", my_serial->rx_pin_);
        ESP_LOGCONFIG(TAG, "  RX Buffer Size: %u", my_serial->get_rx_buffer_size());
        ESP_LOGCONFIG(TAG, "  Baud Rate: %u baud", my_serial->get_baud_rate());
        ESP_LOGCONFIG(TAG, "  Data Bits: %u", my_serial->get_data_bits());
        ESP_LOGCONFIG(TAG, "  Parity: %s", LOG_STR_ARG(parity_to_str(my_serial->get_parity())));
        ESP_LOGCONFIG(TAG, "  Stop bits: %u", my_serial->get_stop_bits());
        // нога индикации активности
        LOG_PIN("Signal active pin ", signal_led);
        // какие команды используем для опроса
        ESP_LOGCONFIG(TAG, "Bilding command line: ' %s ', size:%u", comm, comm_counter); 
    }

    void setup() override{
       createComm(); // построить буфер команд    
       setKey(KEY_MENU); // на всякий случай для сброса устройства
    }  
    
    // берем время из BM8036
    void read_time() {
        if(my_rtc.readTs==0){ // данные еще не получали
            return;
        }
        const uint8_t day_core[]={0, 2,3,4,5,6,7,1}; // счет 1..7, у нас воскр-7, у них воскр-1
        time::ESPTime rtc_time{.second        = (uint8_t)(10u * my_rtc.clock.ten_seconds + my_rtc.clock.seconds), 
                               .minute       = (uint8_t)(10u * my_rtc.clock.ten_minutes + my_rtc.clock.minutes),
                               .hour         = (uint8_t)(10u * my_rtc.clock.ten_hours + my_rtc.clock.hours), 
                               .day_of_week  = (uint8_t)(day_core[my_rtc.clock.day]),
                               .day_of_month = (uint8_t)(10u * my_rtc.clock.ten_date + my_rtc.clock.date), 
                               .day_of_year  = 1, // сей час это пофиг
                               .month        = (uint8_t)(10u * my_rtc.clock.ten_month + my_rtc.clock.month), 
                               .year         = (uint16_t)(2000u + my_rtc.clock.year + 10u * my_rtc.clock.ten_year)};
        rtc_time.recalc_timestamp_utc(false);
        if (!rtc_time.is_valid()) { // при ошибке заканчиваем танцы с бубном вокруг часов
            _debugMsg(F("%010u: Invalid RTC time, not syncing to system clock."), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());                    
            return;
        }
        time_t time_utc0=rtc_time.timestamp-rtc_time.timezone_offset()+(millis()-my_rtc.readTs)/1000; // корректируем к UTC 0 + задержка
        rtc_time.from_epoch_local(time_utc0); 
        time::RealTimeClock::synchronize_epoch_(time_utc0); // передаем время раз все нормально
        _debugMsg(F("%010u: Sync clock to device RTC: %u:%02u:%02u %u/%02u/%u, day of week %u"), ESPHOME_LOG_LEVEL_INFO, __LINE__, millis(),
                      rtc_time.hour, rtc_time.minute, rtc_time.second, rtc_time.day_of_month, rtc_time.month, rtc_time.year , rtc_time.day_of_week);
    }
        
    // устанавливаем время на BM8036
    void write_time() { // поднимем флаг необходимости установки времени на устройстве
        my_rtc.needSet=true; // реально время установится когда возможно будет передать пакет устройству
        _debugMsg(F("%010u: Get job 'write_time'"), ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());
    }
    
    void loop() override {
        // если подключен uart
        if(my_serial!=nullptr){ // будем проверять прием данных
            
            // обработка приема данных 
            if(my_serial->available()){  // если в буфере приема UART есть данные
                set_pin_state(true); // включить светодиод
                while(my_serial->available()){
                   my_serial->read_byte(&(in_data[in_size++])); // получили байт от устройства - кладем в буфер
                   //_debugMsg(F("%010u: R: %02X"), ESPHOME_LOG_LEVEL_ERROR, __LINE__,millis(),in_data[in_size-1]);              
                   if(in_size>=IN_BUFF_SIZE){ // во избежаниии переполнения буфера
                       in_size=IN_BUFF_SIZE-1;    
                   }
                }
                in_last_time=millis(); // засекли время прихода байта
                in_timeout=IN_BYTE_TIMEOUT; //время ожидания между байтами
                out_last_time=in_last_time; // перезасекаем таймер отправки пакета
            } else {
                set_pin_state(false); // вЫключить светодиод
            }
                
            // обработка буфера отправки
            if(out_sended<out_size && millis()-out_last_time>=out_timeout){ // в буфере есть данные и таймаут отправки предыдущего байта истек
                my_serial->write_array(&(out_buff[out_sended++]),1); // отправляем байт 
                //_debugMsg(F("%010u: W: %02X"),ESPHOME_LOG_LEVEL_ERROR, __LINE__,millis(),out_buff[out_sended-1]);              
                out_last_time=millis(); //фиксируем время последней отправки
                in_last_time=out_last_time; //таймер ожидания ответа 
                in_timeout=REPLY_TIMEOUT; //время ожидания пакета
                if(out_sended==out_size){ // данные кончились
                    out_sended=0; // взводим буфер 
                    out_size=0;
                }
            }

            if(out_size==0 && in_last_time==0 && millis()-out_last_time>out_delay){
                // обслуживание очереди отправки кнопок
                if(availableKey()){ // в очереди есть код кнопки для отправки
                   uint8_t buff[2]={'K',getKey()};
                   sendSerialData(buff, 2, SLOW_SERIAL_DELAY); // отправляем код клавиши 
                   ESP_LOGD(TAG,"Send key 0x%02X",buff[1]);
                }
                // если буфер отправки пуст, не ждем никакого ответа, таймаут между отправками истек, отправили еще не весь список запросов
                if(my_rtc.needSet && my_rtc.readTs!=0){ // поднят запрос установки времени на устройстве и структура проинициализирована
                    out_delay=OUT_DELAY; // скорости установки норамльные
                    auto now = time::RealTimeClock::now(); // в устройство льем время в соответствии с временной зоной
                    if (!now.is_valid()) {
                        _debugMsg(F("%010u: Invalid system time, not syncing to NM8036 RTC."), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());
                    } else {
                        const uint8_t day_core[]={0,7,1,2,3,4,5,6}; // счет 1..7, у нас воскр-7, у них воскр-1
                        //const uint8_t day_core[]={0, 2,3,4,5,6,7,1};
                        my_rtc.clock.year = (now.year - 2000) % 10;
                        my_rtc.clock.ten_year = (now.year - 2000) / 10 % 10;
                        my_rtc.clock.month = now.month % 10;
                        my_rtc.clock.ten_month = now.month / 10;
                        my_rtc.clock.date = now.day_of_month % 10;
                        my_rtc.clock.ten_date = now.day_of_month / 10;
                        my_rtc.clock.day = day_core[now.day_of_week];
                        my_rtc.clock.hours = now.hour % 10;
                        my_rtc.clock.ten_hours = now.hour / 10;
                        my_rtc.clock.minutes = now.minute % 10;
                        my_rtc.clock.ten_minutes = now.minute / 10;
                        my_rtc.clock.seconds = now.second % 10;
                        my_rtc.clock.ten_seconds = now.second / 10;
                        sendSerialData('T', SLOW_SERIAL_DELAY);
                        sendSerialData((uint8_t*)&(my_rtc.clock), sizeof(my_rtc.clock), SLOW_SERIAL_DELAY); // этот пакет отправляем медленно  
                        my_rtc.readTs=millis(); // поскольку установили время, правим время считывания
                        _debugMsg(F("%010u: Set time to NM8036 RTC."), ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());
                        _debugPrintPacket(out_buff , out_size, false, ESPHOME_LOG_LEVEL_DEBUG,__LINE__); // отладочная печать буфера отправки данных
                        _debugMsg(F("%010u: Set clock to device RTC: %u:%02u:%02u %u/%02u/%u, day of week %u"), ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis(),
                                    my_rtc.clock.hours   + 10u * my_rtc.clock.ten_hours,
                                    my_rtc.clock.minutes + 10u * my_rtc.clock.ten_minutes,
                                    my_rtc.clock.seconds + 10  * my_rtc.clock.ten_seconds,
                                    my_rtc.clock.date    + 10u * my_rtc.clock.ten_date,
                                    my_rtc.clock.month   + 10u * my_rtc.clock.ten_month,
                                    (uint16_t)2000 + my_rtc.clock.year + 10u * my_rtc.clock.ten_year,
                                    my_rtc.clock.day);
                    }
                }
                    
                if(out_size==0 && send_counter<comm_counter){ //отправки данных не было, переданы еще не все команды, можем опрашивать устройство
                    _debugMsg("%010u: Send request: '"+String((char)comm[send_counter])+"' to NM8036" , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                    sendSerialData(comm[send_counter++]); // отправляем одиночную команду
                    if(send_counter>=comm_counter){ // зацикливание опроса
                       send_counter=0;   
                    }
                    _debugPrintPacket(out_buff , out_size, false,ESPHOME_LOG_LEVEL_DEBUG,__LINE__); // отладочная печать буфера отправки данных
                }      
            }
        }
            
        // проверка приема пакета
        if(in_last_time!=0){
            if( (millis()-in_last_time>in_timeout)||
                (in_size==1 &&(in_data[0]=='K' || in_data[0]=='T')) ||
                (in_size==2 &&(in_data[0]=='d' || in_data[0]=='l' || in_data[0]=='b')) ||
                (in_size==9 && in_data[0]=='s') ||
                (in_size==97 && in_data[0]=='S') ||
                (in_size==257 && in_data[0]=='D')  ){ // истек таймаут приема или есть все признки получения пакета
                uart_error=_OK; // временно установим статус все норм
                if(in_size==0 && in_data[0]!='K' && in_data[0]!='c'){ // ничего не получили, хотя отправили команду
                    uart_error=NO_REPLY; // нет ответа
                    _debugMsg(F("%010u: No reply from NM8036."), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());
                } else { // получен пакет, поняли по таймауту
                    _debugPrintPacket(in_data, in_size, true,ESPHOME_LOG_LEVEL_DEBUG,__LINE__); // отладочная печать пакета
                    // ********************* ПОКАЗВНИЯ ДАТЧИКОВ ТЕМПЕРАТУРЫ *************************
                    if(in_size>2 && in_size==in_data[0]*2+1 && out_buff[0]=='t' ){ // размер для информации о температурах правильный
                        _debugMsg(F("%010u: Get values of thermo sensors") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        uint8_t count=in_data[0]; // количество датчиков
                        if(count>sizeof(T)/sizeof(T[0])){
                            count=sizeof(T)/sizeof(T[0]);    
                        }
                        for(uint8_t i=0;i<count;i++){
                            int16_t temp=((int16_t*)(&(in_data[1])))[i];// получаем показания датчика                         
                            if(temp!=T[i].val || T[i].change){ // показания изменились
                                T[i].val=temp; // запоминаем новые показания
                                if(T[i].sensor!=nullptr){ // ПУБЛИКАЦИЯ 
                                    T[i].change=false; // флаг необходимости публикации данных
                                    T[i].sensor->publish_state(T[i].getTemp()); 
                                }
                            }
                        }
                    // ********************* СТАТУС ДАТЧИКОВ ТЕМПЕРАТУРЫ *************************
                    } else if(in_size>2 && in_size==in_data[0]+1 && out_buff[0]=='v'){ // размер правильный ответ о статусах датчиков
                        _debugMsg(F("%010u: Get status of termometers") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        uint8_t count=in_data[0]; // количество датчиков
                        if(count>sizeof(T)/sizeof(T[0])){
                            count=sizeof(T)/sizeof(T[0]);    
                        }
                        for(uint8_t i=0;i<count;i++){
                            T[i].isOk=(in_data[i+1]!=0);  // состояние датчика
                        }
                    // ********************* СОСТОЯНИЯ ВЫХОДОВ *************************
                    } else if(in_size==2 && out_buff[0]=='l' ){ // размер правильный, ответ о состоянии выходов
                        _debugMsg(F("%010u: Get status of outputs") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        //uint16_t data=*((uint16_t*)in_data);
                        uint16_t data=((uint16_t)in_data[1]<<8) + in_data[0];
                        static bool first_load=true; // флаг первичной загрузки
                        uint16_t mask=(1u<<((sizeof(O)/sizeof(O[0]))-1)); // маска для разбора битовой переменной состояния выходов 11 бит-1 выход, 10 бит-2 выход
                        for(uint8_t i=0;i<sizeof(O)/sizeof(O[0]);i++){
                            bool test=((data&mask)!=0); // определяем состояние бита соответствующего выходу 
                            if(O[i].state!=test || first_load || O[i].change){ // состояние выхода устройства изменилось или первая загрузка
                                O[i].state=test; // запомним текущее состояние выхода
                                if(O[i].sensor!=nullptr){ // ПУБЛИКАЦИЯ
                                   O[i].change=false; // снимем флаг изменения
                                   O[i].sensor->publish_state(test);
                                }
                            }
                            mask>>=1; // готовим маску к проверке сл. бита
                        }
                    // ********************* ДАННЫЕ О СЕРИЙНЫХ НОМЕРАХ *************************
                    } else if(in_size==257 && out_buff[0]=='D'){ // получили данные о серийных номерах датчиков
                        if(in_data[0]=='D'){
                            _debugMsg(F("%010u: Get serial numbers of thermo sensors") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                            for(uint8_t i=0;i<sizeof(T)/sizeof(T[0]);i++){
                                uint8_t* curSn= &(in_data[(8*i)+1]); // адрес расположения серийного номера в буфере   
                                if(memcmp(curSn,T[i].sn,8)!=0 || T[i].snCh){ // номер не совпал
                                    memcpy(T[i].sn,curSn,8); // копируем номер датчика
                                    if(T[i].text_sens!=nullptr){ // ПУБЛИКУЕМ
                                        T[i].snCh=false; // сброс принуительной публикации
                                        char text_buff[18]; // буфер для создания текстовой строки из адреса датчика
                                        sprintf(text_buff, "%02X%02X%02X%02X%02X%02X%02X%02X", T[i].sn[0],T[i].sn[1],T[i].sn[2],T[i].sn[3],T[i].sn[4],T[i].sn[5],T[i].sn[6],T[i].sn[7]);
                                        T[i].text_sens->publish_state(text_buff); // публикуем серийный номер
                                    }
                                }
                            }   
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа  
                        }
                    // ********************* НАПРЯЖЕНИЕ БАТАРЕЙКИ *************************
                    } else if(in_size==2 && out_buff[0]=='b'){ // размер правильный, ответ о напряжении батарейки
                        _debugMsg(F("%010u: Get battery voltage RTC") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        uint16_t data=((int16_t)in_data[1]<<8) | in_data[0];
                        if(Bat.value!=data || Bat.change){ // если изменилось
                            Bat.value=data;
                            if(Bat.sensor!=nullptr){
                                Bat.change=false; // флаг принудительной публиковать
                                Bat.sensor->publish_state(Bat.getVolt());
                            }
                        }
                    // ********************* ВЕРСИЯ ПРОШИВКИ *************************
                    } else if(in_size>2 && out_buff[0]=='V' && in_data[1]+2==in_size){  //размер правильный, ответ о версии
                        if(in_data[0]=='V'){
                            _debugMsg(F("%010u: Get firmware version") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                            if(Vers.buff==nullptr){ // еще не получали версию
                                Vers.buff=(char*) malloc(in_data[1] + 1); // выделяем память под версию
                                if(Vers.buff!=nullptr){ // если удачно, выбираем только цифры
                                    uint8_t j=0;
                                    for(uint8_t i=2;i<in_data[1]+2;i++){
                                        // выбираем только цифровое обозначение, дебильное слово "Версия" - какая то глупость
                                        if((in_data[i]>='0' &&  in_data[i]<='9') || in_data[i]==',' ||
                                            in_data[i]=='.' || in_data[i]=='_' || in_data[i]=='-'){
                                            Vers.buff[j++]=in_data[i]; 
                                        }
                                    }
                                    Vers.buff[j]=0; // конец строки
                                    Vers.sensor->publish_state(Vers.buff); // ПУБЛИКАЦИЯ
                                    free(Vers.buff); // освобождаем память буфера
                                    Vers.buff=nullptr; 
                                    comm[0]='b'; //ЗАМЕНЯЕМ ЗАПРОС ВЕРСИИ ЗАПРОСОМ ЧТЕНИЯ НАПРЯЖЕНИЯ БАТАРЕИ, БОЛЬШЕ ВЕРСИЯ НАМ НЕ НУЖНА !!!!
                                }
                            }
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа 
                        }
                    // ********************* АНАЛОГОВЫЕ ВХОДЫ *************************
                    } else if(in_size==9 && out_buff[0]=='s'){ // размер правильный, ответ о показаниях ADC
                        if(in_data[0]=='s'){
                            _debugMsg(F("%010u: Get ADC values") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                            uint8_t arrow=1;
                            for(uint8_t i=0; i<4; i++){
                                uint16_t temp=(int16_t)in_data[arrow] | (int16_t)(in_data[arrow+1]<<8);
                                arrow+=2;
                                if(Adc[i].value!=temp || Adc[i].change){ // показания ADC изменились
                                    Adc[i].value=temp;
                                    if(Adc[i].sensor!=nullptr){ // ПУБЛИКАЦИЯ
                                       Adc[i].change=false;
                                       Adc[i].sensor->publish_state(temp); //опубликовать значение входа в попугаях
                                    }
                                }
                            }
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа 
                        }
                    // ********************* ЧТЕНИЕ ВНУТРЕННИХ ЧАСОВ *************************
                    } else if(in_size==sizeof(struct_clock)+1 && out_buff[0]=='c' ){ // размер правильный, ответ о времени
                        if( in_data[0]=='c'){
                            struct_clock* clc = (struct_clock*)(&(in_data[1]));
                            _debugMsg(F("%010u Get clock data from device RTC: %u:%02u:%02u %u/%02u/%u, day of week %u"), ESPHOME_LOG_LEVEL_DEBUG, __LINE__,millis(),
                                        clc->hours   + 10u * clc->ten_hours,
                                        clc->minutes + 10u * clc->ten_minutes,
                                        clc->seconds + 10  * clc->ten_seconds,
                                        clc->date    + 10u * clc->ten_date,
                                        clc->month   + 10u * clc->ten_month,
                                        (uint16_t)2000 + clc->year + 10u * clc->ten_year,
                                        clc->day);
                            if(my_rtc.needSet==false || my_rtc.readTs==0){ // если нет запроса на установку времени или время еще ни разу не считали, структура не инициализирована
                                my_rtc.readTs=out_last_time+1; // время отправки запроса о показаниях часов + время на обработку   
                                memcpy(&(my_rtc.clock), &(in_data[1]), sizeof(struct_clock));
                            }
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа 
                        }
                    // ********************* ПОЛУЧИЛИ ПОДТВЕРЖДЕНИЕ УСТАНОВКИ ВРЕМЕНИ *************************
                    } else if(in_size==1 && out_buff[0]=='T'){ // размер правильный, ответ о времени
                        if(in_data[0]=='T'){
                            my_rtc.needSet=false; // снимаем флаг необходимости синхронизации 
                            _debugMsg(F("%010u: RTC clock is set successfully") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа 
                        }
                    // ********************* ПОЛУЧИЛИ ПОДТВЕРЖДЕНИЕ ИЗМЕНЕНИЯ ДИСПЛЕЯ *************************
                    } else if(in_size==2 && out_buff[0]=='d'){ // содержимое дисплея
                        if(in_data[0]=='d'){
                           static uint32_t localTimer=0; // раз в TIMEOUT_FREEZE/2 читаем дисплей принудительно
                           if(in_data[1]==0 && millis()-localTimer<TIMEOUT_FREEZE/2){ // данные дисплея не изменились, перескакиваем через сл. байт запроса ('S')
                               send_counter++;
                               localTimer=millis();
                           }
                        }
                    // ********************* ПОЛУЧИЛИ СОДЕРЖИМОЕ ДИСПЛЕЯ *************************
                    } else if(in_size==97 && out_buff[0]=='S'){ // содержимое дисплея
                        if(in_data[0]=='S'){
                           char* temp=decodeLCDLine(in_data+65, in_data+1);
                           if(memcmp(dispStr1,temp,16)!=0){ // изменилась первая строка
                              memcpy(dispStr1,temp,16); // сохраняем содержимое дисплея
                              freeze_controll_timer=millis(); // таймер контроля зависания, сбрасываем при изменении данных
                              if(displ1!=nullptr){
                                 displ1->publish_state(strUni(temp));
                              }
                           }
                           temp=decodeLCDLine(in_data+65+16, in_data+1);
                           if(memcmp(dispStr2,temp,16)!=0){ // изменилась вторая строка
                              memcpy(dispStr2,temp,16); // сохраняем содержимое дисплея
                              freeze_controll_timer=millis(); // таймер контроля зависания, сбрасываем при изменении данных
                              if(displ2!=nullptr){
                                 displ2->publish_state(strUni(temp));
                              }
                           }
                           //ESP_LOGE("",strUni(dispStr1));
                           //ESP_LOGE("",strUni(dispStr2));
                           //ESP_LOGE("",strUni(currLine));
                           uint8_t deep=0;
                           uint8_t pos=0;
                           getScrPos(&deep, &pos); // опознать состояние дисплея
                           if((inter_switch==nullptr || inter_switch->state==false) && deep!=MENU_UNDEF){ // если отключен перехват управления
                              set_one_key(KEY_MENU); // жмем меню, для выхода
                           } else if(inter_switch!=nullptr && inter_switch->state){ // перехват управления включен
                              if(deep==MENU_UNDEF){ // если рабочая страница - нажимаем кнопку меню, для входа в меню
                                 set_one_key(KEY_MENU); // жмем меню, для выхода 
                              } else if(deep==MENU_FIRST){ // первый уровень меню   
                                if(pos==POS_CLOCK_SET || pos==POS_PROGR_SET || pos==POS_FIND_MEAS || pos==POS_PARAMS || pos==POS_ADC || pos==POS_OUT){
                                   set_one_key(KEY_ARR_DN);
                                } else if(pos==POS_LIGHT || pos==POS_CONTRAST || pos==POS_SOUND || pos==POS_SCREEN  || pos==POS_VERSION){
                                   set_one_key(KEY_ARR_UP);
                                } else if(pos==POS_MANUAL){ //текущая строка - вход в управление 
                                   set_one_key(KEY_ENTER);
                                } else { // при ошибке выйти из меню
                                   set_one_key(KEY_MENU);
                                }
                              } else if(deep==MENU_MANUAL){ // уже в манульном режиме  
                                uint8_t i=pos-POS_MANUAL_1; // детектированный номер переключателя 
                                if(targetManualOut>i){ // движение к требуемой строке
                                   set_one_key(KEY_ARR_DN);   
                                } else if(targetManualOut<i){
                                   set_one_key(KEY_ARR_UP); 
                                } else if(targetManualOut==i){ // выбрана нужная строка
                                   if(i<sizeof(O)/sizeof(O[0])){ // номер в диапазоне
                                      if(O[i].switch_!=nullptr){ // свич инициализирован
                                         bool val=(currLine[14]=='1'); // состояние выхода на экране
                                         if(O[i].switch_->state != val){ // если не равно
                                            set_one_key(KEY_ENTER); // изменяем состояние выхода под требуемое    
                                         } else { // если состояние выхода правильное
                                            if(availableSw()){ // в очереди еще есть свичи требующие изменения
                                               targetManualOut=getSw(); // актуализировать
                                            } else {
                                               out_delay=OUT_DELAY; //пауза между запросами отправки команд к NM8036 в нормальный режим
                                            }                                                
                                          }
                                      }                                         
                                   }
                                } else { // при ошибке выйти из меню
                                   set_one_key(KEY_MENU);
                                }
                              }                              
                           } else if(deep==MENU_UNDEF){
                              out_delay=OUT_DELAY; //пауза между запросами отправки команд к NM8036 в нормальный режим
                           }
                        } else {
                           uart_error=ER_SIZE; // ошибка размера ответа 
                        }
                    // ********************* ПОДТВЕРДИЛИ О ПРИМЕНЕНИИ КНОПКИ *************************
                    } else if(in_size==1 && in_data[0]=='K'){
                        if(out_buff[0]=='K'){ // ответ об обработке кнопки
                           if(availableKey()==false){
                              key_ok=true; // все кнопки отправлены, в буфере нет больше кнопок
                           }
                           send_counter=posS; // для чтения экрана сразу после нажатия кнопки
                        } else {
                           uart_error=ER_SIZE; // ошибка размера ответа 
                        }
                    }
                    in_size=0; // очистим буфер приема пакета
                }
                if(uart_error==_OK){ // считаем ошибки связи
                    freeze_controll_timer=millis();   // контроль зависания
                    error_count=0;
                } else {
                    error_count++;
                    _debugMsg(F("%010u: Сommunication error: %s") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis(),  getStrError(uart_error)); 
                    my_serial->flush();
                }
                in_last_time=0; // отключим ожидание пакета
            } 
            out_last_time=millis(); //засечем паузу перед отправкой пакета
        }
        // раз таймер не 0, значит читаем дисплей, позиция экрана не ручное управление выходами, таймаут истек
        if(freeze_controll_timer && millis()-freeze_controll_timer>TIMEOUT_FREEZE){
           uart_error=ER_FREEZE; // внешнее устройство висит
           //uint8_t buff[2]={'K',KEY_MENU};
           //sendSerialData(buff, 2, SLOW_SERIAL_DELAY); // отправляем код клавиши 
           //ESP_LOGD(TAG,"Send key for unfreze 0x%02X",buff[1]);
        }
    }
    
    // публикуем данные
    void update() override {
   
        static uint32_t last_publish=0;
        uint32_t start_update_time=millis(); // начало процедуры
        if(start_update_time-last_publish>MIN_PUBLISH_PERIOD){ // ограничу период для непонятливых
        
            // проверка состояния выключателей отключем если режим управления не перехвачен
            if(inter_switch!=nullptr &&  inter_switch->state==false){ 
               for(uint8_t i=0;i<sizeof(O)/sizeof(O[0]);i++){           
                  if(O[i].switch_!=nullptr && O[i].switch_->state){
                     O[i].switch_->turn_off();    
                  }
               } 
            }
        
            if(my_serial==nullptr){
                _debugMsg(F("%u10 Fatal error: Uart parameters not defined, check config !"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());
                uart_error=ER_UART_INIT;
            }

            // контролируем ошибки связи и момент их возникновения
            static _errType oldError = _OK; 
            if(oldError != uart_error){ //если изменился статус ошибки
                oldError = uart_error;  // запомним новый статус
                if(error_string!=nullptr){ // публикация ошибок
                    // ОБРАБОТКА ГЛОБАЛЬНОЙ ОШИБКИ СВЯЗИ
                    if(error_count>REBOOT_ERROR_COUNT){ // перезагрузка из-за ошибок связи с mn8036
                        error_string->publish_state(getStrError(ER_REBOOT));
                        _debugMsg(F("%010u: Error: %s"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis(), getStrError(ER_REBOOT));
                        delay(10000);
                        if(my_serial!=nullptr){
                            my_serial->flush();
                        }
                        #ifdef ESP32
                            esp_sleep_enable_timer_wakeup(1000000UL * 30);  //глубокий сон 30 секунд
                            esp_deep_sleep_start();
                        #else
                            ESP.restart(); 
                        #endif
                    } 
                }
                if(oldError==_OK){
                    _debugMsg(F("%010u: No errors !"), ESPHOME_LOG_LEVEL_INFO, __LINE__, millis());
                } else {
                    _debugMsg(F("%010u: Error: %s"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis(), getStrError(oldError));
                }
            }

            if(error_string!=nullptr){
               static uint8_t old=-1;
               if(old!=uart_error){
                  old=uart_error;
                  error_string->publish_state(getStrError(uart_error));
               }
            }
            
        }
    }
        
  friend class mn8036_Switch;
}; //NM8036Component  

template<typename... Ts> class WriteAction : public Action<Ts...>, public Parented<NMBM8036> {
 public:
  void play(Ts... x) override { this->parent_->write_time(); }
};

template<typename... Ts> class ReadAction : public Action<Ts...>, public Parented<NMBM8036> {
 public:
  void play(Ts... x) override { this->parent_->read_time(); }
};

}  // namespace nmbm8036
}  // namespace esphome

#endif

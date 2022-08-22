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
                        ER_UART_INIT  // в ямле не прописан UART
};

#define IN_BUFF_SIZE 512      // размер входного буфера
#define OUT_BUFF_SIZE 32      // размер выходного буфера
#define REPLY_TIMEOUT 5000    //mS максимальный таймаут ожидания начала ответа от NM8036 
#define IN_BYTE_TIMEOUT 60    //mS максимальный таймаут между ожиданием байтов при ответе от NM8036 
#define OUT_DELAY  2000       //mS пауза между запросами отправки команд к NM8036
#define MAX_UPDATE_TIME 250   //mS максимальная продолжительность задержки в update()
#define SLOW_SERIAL_DELAY 5   //mS задержка между байтами при медленной отправке
#define REBOOT_ERROR_COUNT    50 // счетчик ошибок потери связи, до перезагрузки устройства
#define MIN_PUBLISH_PERIOD 1000  // минимально возможный период публикации

class NMBM8036 : public Sensor, public RealTimeClock {
 private:
    const char *const TAG = "nmbm8036";
    sDallas  T[32]; // массив сенсоров температуры
    sChannel O[12]; // массив выходов
    sBattery   Bat; // батарея
    sVersion  Vers; // версия ПО устройства
    sADC    Adc[4]; // показания ADC
    sRTC    my_rtc; // часы устройства
    
    // для контроля и информировании о состоянии соединения
    UARTComponent *my_serial{nullptr};// указатель на UART для подключения к устройству
    TextSensor *error_string {nullptr}; // состояние канала связи RS232 текстовая рашифровка
    uint8_t error_count=0; // счетчик последовательных ошибок, для перезагрузки
    _errType uart_error=_OK; //состояние канала связи RS232  
    GPIOPin* signal_led{nullptr}; // нога с сигнальным диодом
    bool oldPinState = false; // текущее состояние светодиода статуса
        
    // переменные для организации трафика к устройсву
    uint8_t send_counter=0; // счетчик маршрутизатор отправки команд устройству
    uint8_t comm[7]={'V', 'c', 'v', 'D', 't', 'l', 's' }; //последовательность опроса, команды, ПОСЛЕ УДАЧНОГО ЧТЕНИЯ ВЕРСИИ 'V' заменяется на 'b' - чтение батарейки, см. ответ на 'V'
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
    
 public:
    // подключение температурных сенсоров
    void set_temperature_sensor(sensor::Sensor *temperature_sensor, uint8_t num) { if(num<sizeof(T)/sizeof(T[0])) T[num].sensor = temperature_sensor;}
    // подключение сенсоров серийных номеров термосенсоров
    void set_dallas_sn_sensor(text_sensor::TextSensor *SN_sensor, uint8_t num) { if(num<sizeof(T)/sizeof(T[0])) T[num].text_sens = SN_sensor;}
    // подключение сенсоров выходов
    void set_output_sensor(binary_sensor::BinarySensor *out_sensor, uint8_t num) { if(num<sizeof(O)/sizeof(O[0])) O[num].sensor = out_sensor;}
    // подключение сенсоров выходов (ADC) 
    void set_adc_sensor(sensor::Sensor *adc, uint8_t num) { if(num<sizeof(Adc)/sizeof(Adc[0])) Adc[num].sensor = adc;}
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
        // перечисление выходов
        for(uint8_t i=0; i<sizeof(O)/sizeof(O[0]); i++){
            if(O[i].sensor!=nullptr){
                sprintf(textBuf, "Channel %d output ", i); 
                LOG_BINARY_SENSOR("", textBuf, this->O[i].sensor);
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
        
    }
 
    void setup() override{
        // хочу мощный сигнал
        //esp_wifi_set_max_tx_power(80);
        // что бы зарядить страйпы у МАХ3232 (дерьмовая микросхема)
        //uint8_t d='V';
        //for(uint16_t i=0; i<250; i++){
        //    if(my_serial!=nullptr){
        //       my_serial->write_array(&d,1); // отправляем байт
        //    }
        //}       
    }
    
    // берем время из BM8036
    void read_time() {
        if(my_rtc.readTs==0){ // данные еще не получали
            return;
        }
        time::ESPTime rtc_time{.second        = (uint8_t)(10u * my_rtc.clock.ten_seconds + my_rtc.clock.seconds), 
                               .minute       = (uint8_t)(10u * my_rtc.clock.ten_minutes + my_rtc.clock.minutes),
                               .hour         = (uint8_t)(10u * my_rtc.clock.ten_hours + my_rtc.clock.hours), 
                               .day_of_week  = my_rtc.clock.day,
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
        _debugMsg(F("%010u: Sync clock to device RTC: %u:%02u:%02u %u/%02u/%u, day of week %u"), ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis(),
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

            if(out_size==0 && in_last_time==0 && millis()-out_last_time>OUT_DELAY){
                // если буфер отправки пуст, не ждем никакого ответа, таймаут между отправками истек, отправили еще не весь список запросов
                if(my_rtc.needSet && my_rtc.readTs!=0){ // поднят запрос установки времени на устройстве и структура проинициализирована
                    auto now = time::RealTimeClock::now(); // в устройство льем время в соответствии с временной зоной
                    if (!now.is_valid()) {
                        _debugMsg(F("%010u: Invalid system time, not syncing to NM8036 RTC."), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());
                    } else {
                        my_rtc.clock.year = (now.year - 2000) % 10;
                        my_rtc.clock.ten_year = (now.year - 2000) / 10 % 10;
                        my_rtc.clock.month = now.month % 10;
                        my_rtc.clock.ten_month = now.month / 10;
                        my_rtc.clock.date = now.day_of_month % 10;
                        my_rtc.clock.ten_date = now.day_of_month / 10;
                        my_rtc.clock.day = now.day_of_week;
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
                        _debugPrintPacket(out_buff , out_size, false); // отладочная печать буфера отправки данных
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
                    
                if(out_size==0 && send_counter<sizeof(comm)){ //отправки данных не было, переданы еще не все команды, можем опрашивать устройство
                    _debugMsg("%010u: Send request: '"+String((char)comm[send_counter])+"' to NM8036" , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                    sendSerialData(comm[send_counter++]); // отправляем одиночную команду
                    _debugPrintPacket(out_buff , out_size, false); // отладочная печать буфера отправки данных
                }      
            }
        }
            
        // проверка приема пакета
        if(in_last_time){
            if(millis()-in_last_time>in_timeout){ // истек таймаут приема
                uart_error=_OK; // временно установим статус все норм
                if(in_size==0){ // ничего не получили, хотя отправили команду
                    uart_error=NO_REPLY; // нет ответа
                    _debugMsg(F("%010u: No reply from NM8036."), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());
                } else { // получен пакет, поняли по таймауту
                    _debugPrintPacket(in_data, in_size, true); // отладочная печать пакета
                    if(in_size>2 && in_size==in_data[0]*2+1 && out_buff[0]=='t' ){ // размер для информации о температурах правильный
                        _debugMsg(F("%010u: Get values of thermo sensors") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        uint8_t count=in_data[0]; // количество датчиков
                        if(count>sizeof(T)/sizeof(T[0])){
                            count=sizeof(T)/sizeof(T[0]);    
                        }
                        for(uint8_t i=0;i<count;i++){
                            int16_t temp=((int16_t*)(&(in_data[1])))[i];// получаем показания датчика                         
                            if(temp!=T[i].val){ // показания изменились
                                T[i].val=temp;  // запоминаем новые показания
                                T[i].change=true; // флаг необходимости публикации данных
                            }
                        }
                    } else if(in_size>2 && in_size==in_data[0]+1 && out_buff[0]=='v'){ // размер правильный ответ о статусах датчиков
                        _debugMsg(F("%010u: Get status of termometers") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        uint8_t count=in_data[0]; // количество датчиков
                        if(count>sizeof(T)/sizeof(T[0])){
                            count=sizeof(T)/sizeof(T[0]);    
                        }
                        for(uint8_t i=0;i<count;i++){
                            T[i].isOk=(in_data[i+1]!=0);  // состояние датчика
                        }
                    } else if(in_size==2 && out_buff[0]=='l' ){ // размер правильный, ответ о состоянии выходов
                        _debugMsg(F("%010u: Get status of outputs") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        uint16_t data=((int16_t)in_data[1]<<8) | in_data[0];
                        static bool first_load=true; // флаг первичной загрузки
                        uint16_t mask=(1u<<sizeof(O)/sizeof(O[0])); // маска для разбора битовой переменной состояния выходов 0 бит-12 выход, 11 бит-1 выход
                        for(uint8_t i=0;i<sizeof(O)/sizeof(O[0]);i++){
                            bool test=((data&mask)!=0); // определяем состояние бита соответствующего выходу 
                            if(O[i].state!=test || first_load){ // состояние выхода устройства изменилось или первая загрузка
                                O[i].change=true; // поднимем флаг изменения
                                O[i].state=test; // запомним текущее состояние выхода
                            }
                            mask>>=1; // готовим маску к проверке сл. бита
                        }
                        first_load=false; //данные о состояния входов уже проинициализированы
                    } else if(in_size==257 && out_buff[0]=='D'){ // получили данные о серийном номере
                        if(in_data[0]=='D'){
                            _debugMsg(F("%010u: Get serial numbers of thermo sensors") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                            for(uint8_t i=0;i<sizeof(T)/sizeof(T[0]);i++){
                                uint8_t* curSn= &(in_data[(8*i)+1]); // адрес расположения серийного номера в буфере   
                                if(memcmp(curSn,T[i].sn,8)!=0){ // номер не совпал
                                    memcpy(T[i].sn,curSn,8); // копируем номер датчика
                                    T[i].snCh=true; // можно публиковать этот номер
                                }
                            }   
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа  
                        }
                    } else if(in_size==2 && out_buff[0]=='b'){ // размер правильный, ответ о напряжении батарейки
                        _debugMsg(F("%010u: Get battery voltage RTC") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        uint16_t data=((int16_t)in_data[1]<<8) | in_data[0];
                        if(Bat.value!=data){ // если изменилось
                            Bat.change=true; // нужно публиковать
                            Bat.value=data;
                        }
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
                                    comm[0]='b'; //ЗАМЕНЯЕМ ЗАПРОС ВЕРСИИ ЗАПРОСОМ ЧТЕНИЯ НАПРЯЖЕНИЯ БАТАРЕИ, БОЛЬШЕ ВЕРСИЯ НАМ НЕ НУЖНА !!!!
                                }
                            }
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа 
                        }
                    } else if(in_size==9 && out_buff[0]=='s'){ // размер правильный, ответ о показаниях ADC
                        if(in_data[0]=='s'){
                            _debugMsg(F("%010u: Get ADC values") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                            uint8_t arrow=1;
                            for(uint8_t i=0; i<4; i++){
                                uint16_t temp=(int16_t)in_data[arrow] | (int16_t)(in_data[arrow+1]<<8);
                                arrow+=2;
                                if(Adc[i].value!=temp){ // показания ADC изменились
                                    Adc[i].value=temp;
                                    Adc[i].change=true; // нужно публиковать
                                }
                            }
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа 
                        }
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
                    } else if(in_size==1 && out_buff[0]=='T'){ // размер правильный, ответ о времени
                        if(in_data[0]=='T'){
                            my_rtc.needSet=false; // снимаем флаг необходимости синхронизации 
                            _debugMsg(F("%010u: RTC clock is set successfully") , ESPHOME_LOG_LEVEL_DEBUG, __LINE__, millis());              
                        } else {
                            uart_error=ER_REPLY; // ошибка признака ответа 
                        }                            
                    } else {
                        uart_error=ER_SIZE; // ошибка размера ответа 
                    }
                    in_size=0; // очистим буфер приема пакета
                }
                if(uart_error==_OK){ // считаем ошибки связи
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
    }
    
    // публикуем данные
    void update() override {
    
        static uint32_t last_publish=0;
        uint32_t start_update_time=millis(); // начало процедуры
        if(start_update_time-last_publish>MIN_PUBLISH_PERIOD){ // ограничу период для непонятливых
        
            if(my_serial==nullptr){
                _debugMsg(F("%u10 Fatal error: Uart parameters not defined, check config !"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, millis());
                uart_error=ER_UART_INIT;
            }
        
            static uint8_t oCount=0; // циклический счетчик публикации параметров выходов
            static uint8_t sCount=0; // циклический счетчик публикации параметров датчиков
            static uint8_t aCount=0; // циклический счетчик публикации параметров входов
            
            // перебор датчиков температуры
            while(sCount<sizeof(T)/sizeof(T[0])){
                if(T[sCount].sensor!=nullptr || T[sCount].text_sens!=nullptr){ // сущность есть, будем работать
                    if(T[sCount].isOk){ // датчик подключен к устройству
                        T[sCount].oldIsOk=true; // запоминаем состояние для контроля
                        if(T[sCount].snCh && T[sCount].text_sens!=nullptr){ // получили новый серийный номер и есть куда его публиковать
                            uint8_t test[8]={0};
                            if(memcmp(test,T[sCount].sn,8)==0){ // не правильные данные
                                T[sCount].text_sens->publish_state("unavailable"); // отмечаем, что датчик потерян    
                            } else {
                                char text_buff[18]; // буфер для создания текстовой строки из адреса датчика
                                sprintf(text_buff, "%02X%02X%02X%02X%02X%02X%02X%02X", T[sCount].sn[0],T[sCount].sn[1],T[sCount].sn[2],T[sCount].sn[3],T[sCount].sn[4],T[sCount].sn[5],T[sCount].sn[6],T[sCount].sn[7]);
                                T[sCount].text_sens->publish_state(text_buff); // публикуем серийный номер
                            }
                            T[sCount].snCh=false; // снимаем флаг необходимости публикации серийного номера
                        }
                        if(T[sCount].change && T[sCount].sensor!=nullptr){ // температура изменилась и есть куда ее публиковать
                            T[sCount].sensor->publish_state(T[sCount].getTemp()); // публикуем температуру 
                            T[sCount].change=false; //снимаем флаг изменения температуры
                        }
                    } else if(T[sCount].oldIsOk==true){ // датчик был подключен, но в данный момент не доступен
                        T[sCount].oldIsOk=false; // отмечаем, обработку этого события
                        if(T[sCount].text_sens!=nullptr){
                           T[sCount].text_sens->publish_state("unavailable"); // отмечаем, что датчик потерян
                        }
                        memset(T[sCount].sn,0,8); // удаляем серийный номер для публикации при восстановлении связи с датчиком
                        T[sCount].snCh=false; // снимаем флаг необходимости публикации серийного номера, на всякий случай
                    }                       
                }
                sCount++;
                if(millis()-start_update_time>=MAX_UPDATE_TIME){ // если сидим долго в этой подпрограмме - пока закончим
                    return;
                }
            }
            
            // перебор состояния выходов
            while(oCount<sizeof(O)/sizeof(O[0])){
                if(O[oCount].sensor!=nullptr && O[oCount].change){ // сенсор подключен к ESPHOME, и его состояние поменялось
                    O[oCount].change=false; // сбросить флаг публикации
                    O[oCount].sensor->publish_state(O[oCount].state); //опубликовать состояние датчика
                }
                oCount++;
                if(millis()-start_update_time>=MAX_UPDATE_TIME){ // если сидим долго в этой подпрограмме - пока закончим
                    return;
                }
            }

            // публикация состояния входов
            while(aCount<sizeof(Adc)/sizeof(Adc[0])){
                if(Adc[aCount].sensor!=nullptr && Adc[aCount].change){ // сенсор подключен к ESPHOME, и его состояние поменялось
                    Adc[aCount].change=false; // сбросить флаг публикации
                    Adc[aCount].sensor->publish_state(Adc[aCount].value); //опубликовать значение входа в попугаях
                }
                aCount++;
                if(millis()-start_update_time>=MAX_UPDATE_TIME){ // если сидим долго в этой подпрограмме - пока закончим
                    return;
                }
            }

            // напряжение батареи
            if(Bat.sensor!=nullptr && Bat.change){
                Bat.change=false;
                Bat.sensor->publish_state(Bat.getVolt());
            }

            // версия прошивки
            if(Vers.buff!=nullptr && Vers.sensor!=nullptr){
                Vers.sensor->publish_state(Vers.buff);
                free(Vers.buff); // освобождаем память буфера
                Vers.buff=nullptr;
                //Vers.sensor=nullptr;  // нам этот сенсор больше не нужен
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
            if(millis()-start_update_time>=MAX_UPDATE_TIME){ // если сидим долго в этой подпрограмме - пока закончим
                return;
            }
            
            // запуск цикла опроса
            if(send_counter>=sizeof(comm)){
                send_counter=0;   
            }
            
            if(error_string!=nullptr){
               error_string->publish_state(getStrError(uart_error));
            }
            
            // в сл. раз циклы по новой
            oCount=0;
            sCount=0;
            aCount=0;
        }
    }
        

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

#ifndef LoRaMESH_h
#define LoRaMESH_h

#include <Stream.h>
#include <Arduino.h>

#define MAX_PAYLOAD_SIZE     232
#define MAX_BUFFER_SIZE      237

#define BW125                0x00
#define BW250                0x01
#define BW500                0x02

#define SF7                  0x07
#define SF8                  0x08
#define SF9                  0x09
#define SF10                 0x0A
#define SF11                 0x0B
#define SF12                 0x0C
#define SF_FSK               0x00

#define CR4_5                0x01
#define CR4_6                0x02
#define CR4_7                0x03
#define CR4_8                0x04

#define CLASS_A              0x00
#define CLASS_C              0x02

#define WINDOW_5s            0x00
#define WINDOW_10s           0x01
#define WINDOW_15s           0x02

#define GPIO_MODE_READ       0x00
#define GPIO_MODE_WRITE      0x01
#define GPIO_MODE_CONFIG     0x02

#define LoRa_NOT_PULL        0x00
#define LoRa_PULLUP          0x01
#define LoRa_PULLDOWN        0x02

#define INOUT_DIGITAL_INPUT  0x00
#define INOUT_DIGITAL_OUTPUT 0x01
#define INOUT_ANALOG_INPUT   0x03


#ifndef INPUT
#define INPUT                0
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP         7
#endif

#ifdef  INPUT_PULLDOWN_16
#define INPUT_PULLDOWN       INPUT_PULLDOWN_16
#endif

#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN       8
#endif

class LoRaMESH{
    private:
        bool analogEnabled = false;
    public:
        bool debug_serial = false;
        typedef struct
        {
            uint8_t buffer[MAX_BUFFER_SIZE];
            uint8_t size;
            bool command;
        } Frame_Typedef;

        uint8_t bufferPayload[MAX_PAYLOAD_SIZE] = {0};
        uint8_t payloadSize = 0;

        uint16_t localId = 0;
        uint32_t localUniqueId;
        uint8_t command;
        bool isMaster;

        Frame_Typedef frame;
        uint16_t deviceId = -1;
        uint16_t deviceNet = -1;
        uint32_t deviceUniqueId = -1;

        uint32_t registered_password;

        uint8_t BW, SF, CR, LoRa_class, LoRa_window;

        Stream*  SerialLoRa;
        Stream*  SerialLoRat;

        LoRaMESH(Stream *_SerialLoRa, Stream *_SerialLoRat = NULL){
            SerialLoRa = _SerialLoRa;
            SerialLoRat = _SerialLoRat;
        }

        uint16_t ComputeCRC(uint8_t* data_in, uint16_t length){
            uint16_t i;
            uint8_t bitbang, j;
            uint16_t crc_calc;

            crc_calc = 0xC181;
            for(i=0; i<length; i++)
            {
                crc_calc ^= (((uint16_t)data_in[i]) & 0x00FF);
                
                for(j=0; j<8; j++)
                {
                    bitbang = crc_calc;
                    crc_calc >>= 1;
                    
                    if(bitbang & 1)
                    {
                        crc_calc ^= 0xA001;
                    }
                }
            }
            return (crc_calc&0xFFFF);
        }

        bool prepareFrameCommand(uint16_t id, uint8_t command, uint8_t* payload, uint8_t payloadSize){
            if((id < 0)) return false;
            if(command < 0) return false;
            if(payload < 0) return false;
            if(payloadSize < 0) return false;
            

            uint16_t crc = 0;

            frame.size = payloadSize + 5;
            
            frame.buffer[0] = id&0xFF;
            frame.buffer[1] = (id>>8)&0xFF;
            
            frame.buffer[2] = command;
            
            if((payloadSize >= 0) && (payloadSize < MAX_PAYLOAD_SIZE))
            {
                memcpy(&(frame.buffer[3]), payload, payloadSize);
            
                crc = ComputeCRC((&frame.buffer[0]), payloadSize+3);
                frame.buffer[payloadSize+3] = crc&0xFF;
                frame.buffer[payloadSize+4] = (crc>>8)&0xFF;
            }
            else
            {
                memset(&frame.buffer[0], 0, MAX_BUFFER_SIZE);
                return false;
            }

            frame.command = true;

            return sendPacket();
        }

        bool PrepareFrameTransp(uint16_t id, uint8_t* payload, uint8_t payloadSize)
        {
            uint8_t i = 0;

            if(payload == NULL) return false;
            if(id > 1023) return false;
            if(deviceId == -1) return false;
            
            frame.size = payloadSize + 2;
            frame.buffer[i++] = id&0xFF;
            frame.buffer[i++] = (id>>8)&0x03;
            
            /*
            if((id != 0) && (deviceId == 0)) 
            {
                frame.size = payloadSize + 2;
                frame.buffer[i++] = id&0xFF;
                frame.buffer[i++] = (id>>8)&0x03;
            }
            else
            {
                frame.size = payloadSize;
            }*/
            
            if((payloadSize >= 0) && (payloadSize < MAX_PAYLOAD_SIZE))
            {
                /* Loads the payload */
                memcpy(&frame.buffer[i], payload, payloadSize);
            }
            else
            {
                /* Invalid payload size */
                memset(&frame.buffer[0], 0, MAX_BUFFER_SIZE);
                return false;
            }

            frame.command = false;
            return true;
        }

        bool sendPacket(){
            if(frame.size == 0) return false;
            if(debug_serial)
            {
                Serial.print("TX: ");
                printHex(frame.buffer, frame.size);
                
            }

            if(frame.command)
                SerialLoRa->write(frame.buffer, frame.size);
            else
                if(SerialLoRat == NULL)
                    return false;
                else
                    SerialLoRat->write(frame.buffer, frame.size);

            return true;
        }

        bool receivePacketCommand(uint16_t* id, uint8_t* command, uint8_t* payload, uint8_t* payloadSize, uint32_t timeout){
            uint16_t waitNextByte = 500;
            uint8_t i = 0;
            uint16_t crc = 0;

            if(id < 0x00) return false;
            if(command < 0x00) return false;
            if(payload < 0x00) return false;
            if(payloadSize < 0x00) return false;

            while( ((timeout > 0 ) || (i > 0)) && (waitNextByte > 0) )
            {
                if(SerialLoRa->available() > 0)
                {
                    byte a = SerialLoRa->read();
                    frame.buffer[i++] = a;
                    waitNextByte = 200;
                }
                
                if(i > 0){
                    waitNextByte--;
                }
                timeout--;
                delay(1);
            }
            
            if(debug_serial && i > 0){
                Serial.print("RX: ");
                printHex(frame.buffer, i);
            }

            if((timeout == 0) && (i == 0)) return false;
            crc = (uint16_t)frame.buffer[i-2] | ((uint16_t)frame.buffer[i-1] << 8);
            if(ComputeCRC(&frame.buffer[0], i-2) != crc) return false;
            *id = (uint16_t)frame.buffer[0] | ((uint16_t)frame.buffer[1] << 8);
            *command = frame.buffer[2];
            *payloadSize = i-5;
            memcpy(payload, &frame.buffer[3], i-5);
            
            return true;
        }

        bool receivePacketTransp(uint16_t* id, uint8_t* payload, uint8_t* payloadSize, uint32_t timeout)
        {
            uint16_t waitNextByte = 500;
            uint8_t i = 0;
            
            /* Assert parameters */
            /*
            if((id == NULL) && (deviceId == 0)) return false;
            if(payload == NULL) return false;
            if(payloadSize == NULL) return false;
            if(deviceId == -1) return false;
            */
            /* Waits for reception */
            while( ((timeout > 0 ) || (i > 0)) && (waitNextByte > 0) )
            {
                if(SerialLoRat->available() > 0)
                {
                    frame.buffer[i++] = SerialLoRat->read();
                    waitNextByte = 500;
                }
                
                if(i > 0)
                {
                waitNextByte--;
                }
                timeout--;
                delay(1);
            }

            /* In case it didn't get any data */
            if((timeout == 0) && (i == 0)) return false;

            if(deviceId == 0)
            {
                /* Copies ID */
                *id = (uint16_t)frame.buffer[0] | ((uint16_t)frame.buffer[1] << 8);
                /* Copies payload size */
                *payloadSize = i-2;
                /* Copies payload */
                memcpy(payload, &frame.buffer[3], i-2);
            }
            else
            {
                /* Copies payload size */
                *payloadSize = i;
                /* Copies payload */
                memcpy(payload, &frame.buffer[0], i);
            }
            
            return sendPacket();
        }



        bool localRead(){
            uint8_t b = 0;
            
            bufferPayload[b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;

            prepareFrameCommand(0, 0xE2, bufferPayload, b + 1);
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                if(command == 0xE2)
                {
                    registered_password = bufferPayload[1] << 8;
                    registered_password += bufferPayload[0];
                    
                    localUniqueId = bufferPayload[2];
                    localUniqueId = bufferPayload[3] + (localUniqueId << 8);
                    localUniqueId = bufferPayload[4] + (localUniqueId << 8);
                    localUniqueId = bufferPayload[5] + (localUniqueId << 8);
                    
                    return true;
                }
            } 
            return false;
        }

        void begin(bool _debug_serial = false){
            debug_serial = _debug_serial;
            localRead();
            localId = (uint16_t)frame.buffer[0] | ((uint16_t)frame.buffer[1] << 8);
        }

        bool setNetworkID(uint16_t id){
            uint8_t b = 0;
            
            bufferPayload[b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = (uint8_t)(localUniqueId >> 24);
            bufferPayload[++b] = (uint8_t)(localUniqueId >> 16);
            bufferPayload[++b] = (uint8_t)(localUniqueId >> 8);
            bufferPayload[++b] = (uint8_t)(localUniqueId & 0xFF);
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x04;

            prepareFrameCommand(id, 0xCA, bufferPayload, b + 1);
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
            {
                if(command == 0xCA)
                return true;
            } 
            return false;
        }

        bool setPassword(uint32_t password){
            if(password < 0x00 || password < 0x00 || password > 0xFFFFFFFF)
                return false;
            uint8_t b = 0;

            uint32_t buffer_password; 
            
            bufferPayload[b] = 0x04;
            bufferPayload[++b] = password % 256;
            bufferPayload[++b] = (password / 256) & 0xFF;
            bufferPayload[++b] = ((password / 256) >> 8) & 0xFF;
            bufferPayload[++b] = ((password / 256) >> 16) & 0xFF;
            
            prepareFrameCommand(localId, 0xCD, bufferPayload, b + 1);

            buffer_password = bufferPayload[2] << 8;
            buffer_password += bufferPayload[1];
                
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
            {
                if(command == 0xCD)
                {
                    localRead();
                    
                    if(buffer_password == registered_password)
                        return true;
                }
            }
            
            return false;  
        }

        bool getBPS(bool ignore_cmd = false){
            uint8_t b = 0;

            if(!ignore_cmd){
                bufferPayload[b] = 0x00;
                bufferPayload[++b] = 0x01;
                bufferPayload[++b] = 0x00;

                prepareFrameCommand(localId, 0xD6, bufferPayload, b + 1);
            }

            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                if(command == 0xD6){
                    BW = bufferPayload[2];
                    SF = bufferPayload[3];
                    CR = bufferPayload[4];
                    return true;
                }
            }
            return false;
        }

        bool getClass(bool ignore_cmd = false){
            uint8_t b = 0;
            if(!ignore_cmd){
                bufferPayload[b] = 0x00;
                bufferPayload[++b] = 0xFF;
                bufferPayload[++b] = 0x00;
                bufferPayload[++b] = 0x00;

                prepareFrameCommand(localId, 0xC1, bufferPayload, b + 1);
            }
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                if(command == 0xC1){
                    LoRa_class = bufferPayload[2];
                    LoRa_window = bufferPayload[3];
            
                    return true;
                }
            }
            return false;
        }

        bool setBPS(uint8_t bandwidth = BW500, uint8_t spreading_factor = SF7, uint8_t coding_rate = CR4_5){
            if(bandwidth < 0x00 || bandwidth > 0x02)
                return false;
            else if(spreading_factor < 0x00 || spreading_factor > 0x00 && spreading_factor < 0x07 || spreading_factor > 0x0C)
                return false;
            else if(coding_rate < 0x00 || coding_rate < 0x01 || coding_rate > 0x04)
                return false;

            uint8_t b = 0;

            bufferPayload[b] = 0x01;
            bufferPayload[++b] = 0x14;
            bufferPayload[++b] = bandwidth;
            bufferPayload[++b] = spreading_factor;
            bufferPayload[++b] = coding_rate;
            
            prepareFrameCommand(localId, 0xD6, bufferPayload, b + 1);

            getBPS(true);

            if(BW == bandwidth && SF == spreading_factor && CR == coding_rate)
                return true;

            return false;
        }

        bool setClass(uint8_t lora_class = CLASS_C, uint8_t lora_window = WINDOW_5s){
            if(lora_class < 0x00 || lora_class != 0x00 && lora_class != 0x02)
                return false;
            else if(lora_window < 0x00 || lora_window > 0x02)
                return false;
            
            uint8_t b = 0;

            bufferPayload[b] = 0x00;
            bufferPayload[++b] = lora_class;
            bufferPayload[++b] = lora_window;
            bufferPayload[++b] = 0x00;

            prepareFrameCommand(localId, 0xC1, bufferPayload, b + 1);
            getClass(true);

            if(LoRa_class == lora_class && LoRa_window == lora_window)
                return true;
            
            return false;
        }    

        bool pinMode(uint8_t id, uint8_t gpio, uint8_t inout, uint8_t logical_level = LOW){
            if(gpio < 0x00 || gpio > 0x07)
                return false;
            /*else if(pull < 0x00 || pull > 0x02)
                return false;*/
            else if(inout < 0x00 || inout == 0x02 || inout == 0x03 && gpio < 0x05 && inout == 0x03 && gpio > 0x06 || inout > 0x03)
                return false;
            else if(logical_level < 0x00 || logical_level > 0x03)
                return false;
            uint8_t pull;

            if(inout == INPUT){
              pull = LoRa_NOT_PULL;

            }
            switch(inout){
                case INPUT:          inout = INOUT_DIGITAL_INPUT; pull = LoRa_NOT_PULL; break;
                case INPUT_PULLUP:   inout = INOUT_DIGITAL_INPUT; pull = LoRa_PULLUP; break;
                case INPUT_PULLDOWN: inout = INOUT_DIGITAL_INPUT; pull = LoRa_PULLDOWN; break;
            }

            uint8_t b = 0;
            if(gpio > 4 && gpio < 7 && inout == INOUT_DIGITAL_INPUT){
                bufferPayload[b] = 0x02;
                bufferPayload[++b] = 0x00;
                bufferPayload[++b] = gpio;
                bufferPayload[++b] = 0x00;
                bufferPayload[++b] = INOUT_ANALOG_INPUT;
                analogEnabled = true;
            }
            else{
                bufferPayload[b] = 0x02;
                bufferPayload[++b] = gpio;
                bufferPayload[++b] = pull;
                bufferPayload[++b] = inout;
                bufferPayload[++b] = logical_level;
            }

            prepareFrameCommand(id, 0xC2, bufferPayload, b + 1);
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
            {
                if(command == 0xC2){
                return true;
                }
            }

            return false;
        }

        void getGPIOStatus(int16_t id, uint8_t gpio){
            uint8_t b = 0;

            bufferPayload[b] = 0x00;
            bufferPayload[++b] = gpio;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;

            prepareFrameCommand(id, 0xC2, bufferPayload, b + 1);
        }

        uint16_t analogRead(int16_t id, uint8_t gpio){
            for(uint8_t i = 0; i < 3; ++i){
                getGPIOStatus(id, gpio);
                //0100C2 00 00 05 0A 79 1F0D
                if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
                {
                    if(command == 0xC2){
                        uint16_t rawAnalog = bufferPayload[3]  << 8;
                        rawAnalog |= bufferPayload[4];
                        return rawAnalog;
                    }
                }
            }
            return 0;
        }

        uint8_t digitalRead(int16_t id, uint8_t gpio){
            getGPIOStatus(id, gpio);
            if(analogEnabled && gpio > 4 && gpio < 7){
                if(analogRead(id, gpio) >= 4096 / 2)
                    return 1;
                return 0;
            }
            else{
                if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
                    if(command == 0xC2)
                        return bufferPayload[4];
            }
            return 0;
        } 

        bool digitalWrite(int16_t id, uint8_t gpio, uint8_t logical_level){
            if(gpio < 0x00 || gpio > 0x07)
                return false;
            else if(logical_level < 0x00 || logical_level > 0x03)
                return false;
            else if(analogEnabled && gpio > 4 && gpio < 7)
                return false;

            for(uint8_t i = 0; i < 3; ++i){
                uint8_t b = 0;

                bufferPayload[b] = 0x01;
                bufferPayload[++b] = gpio;
                bufferPayload[++b] = logical_level;
                bufferPayload[++b] = 0x00;

                prepareFrameCommand(id, 0xC2, bufferPayload, b + 1);

                if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                    if(command == 0xC2 && bufferPayload[2] == gpio && bufferPayload[3] == logical_level)
                        return true;
                }
            }

            return false;
        }

        int getNoise(uint8_t id, uint8_t select = 1){
            uint8_t b = 0;
            bufferPayload[b] = 0;
            bufferPayload[++b] = 0;
            bufferPayload[++b] = 0;

            if(select > 2)
                select = 2;

            prepareFrameCommand(id, 0xD8, bufferPayload, b + 1);
            delay(100);
            uint8_t cmd = 0;
            
            if(receivePacketCommand(&localId, &cmd, bufferPayload, &payloadSize, 270)){
                if(cmd == 0xD8)
                    return bufferPayload[select];
                else
                    return 255;
            }
            else
                delay(100);
                
            return 255;
        }

        int getR1(uint16_t rawADC, int R2){
            return  R2 * (4096 - rawADC) / rawADC;
        }

        double getTemp(uint16_t rawADC, int beta, int Rt = 10000, int R2 = 10000){
            double R1 = getR1(rawADC, R2);
            double rx = Rt * exp(-beta/(273.0 + 25.0));
            double t = beta / log(R1/rx);
            return t - 273;
        }
        
        void printHex(uint8_t* num, uint8_t tam){
            char hexCar[4];
            uint8_t index;
            
            for(index = 0; index < tam - 1; index++)
            {
                sprintf(hexCar, "%02X", num[index]);
                Serial.print(hexCar);
            }
            
            sprintf(hexCar, "%02X", num[index]);
            Serial.println(hexCar);  
        }
};
#endif

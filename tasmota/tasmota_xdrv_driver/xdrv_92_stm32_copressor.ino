/*
    xdrv_45_shelly_dimmer.ino - shelly dimmer support for Tasmota

    Copyright (C) 2021  James Turton

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_STM32_COPROCESSOR

/*********************************************************************************************\
 * Shelly WiFi Dimmer v1 and v2 (ESP8266 w/ separate co-processor dimmer)
 *
 * {"NAME":"STM32 CoPr","GPIO":[0,3200,0,3232,5568,5600,0,0,192,0,193,288,0,4736],"FLAG":0,"BASE":18}
 * {"NAME":"STM32 CoPr","GPIO":[0,3200,0,3232,5568,5600,0,0,193,0,192,0,320,4736],"FLAG":0,"BASE":18}
 *
\*********************************************************************************************/

#define XDRV_92                     92

#define STM32_COPROCESSOR_DEBUG
#define STM_LOGNAME                 "STM: "
#define STM_BUFFER_SIZE             256
#define STM_START_BYTE              0x01

#include <stm32flash.h>

#include <TasmotaSerial.h>

TasmotaSerial *StmSerial = nullptr;

struct STM
{
    uint8_t *buffer = nullptr;          // Serial receive buffer
    int byte_counter = 0;               // Index in serial receive buffer
    bool present = false;
} Stm;

struct STM_COMMAND {
  uint8_t command;
  uint8_t parameter;
  uint8_t unused2;
  uint8_t unused3;
} STMCommand;

/*********************************************************************************************\
 * STM firmware update Functions
\*********************************************************************************************/

void StmResetToDFUMode()
{
#ifdef STM32_COPROCESSOR_DEBUG
    AddLog(LOG_LEVEL_DEBUG, PSTR(STM_LOGNAME "Request co-processor reset in dfu mode"));
#endif  // STM32_COPROCESSOR_DEBUG

    pinMode(Pin(GPIO_STM32_RST_INV), OUTPUT);
    digitalWrite(Pin(GPIO_STM32_RST_INV), LOW);

    pinMode(Pin(GPIO_STM32_BOOT0), OUTPUT);
    digitalWrite(Pin(GPIO_STM32_BOOT0), HIGH);

    delay(50);

    // clear in the receive buffer
    while (Serial.available())
        Serial.read();

    digitalWrite(Pin(GPIO_STM32_RST_INV), HIGH); // pull out of reset
    delay(50); // wait 50ms fot the co-processor to come online
}

bool StmUpdateFirmware(uint8_t* data, uint32_t size)
{
#ifdef STM32_COPROCESSOR_DEBUG
    AddLog(LOG_LEVEL_DEBUG, PSTR(STM_LOGNAME "Update firmware"));
#endif  // STM32_COPROCESSOR_DEBUG

    bool ret = true;
    stm32_t *stm = stm32_init(&Serial, STREAM_SERIAL, 1);   // works
    // stm32_t *stm = stm32_init(StmSerial, STREAM_SERIAL, 1);    // does not work
    if (stm)
    {
        off_t   offset = 0;
        uint8_t   buffer[256];
        unsigned int  len;
        const uint8_t *p_st = data;
        uint32_t  addr, start, end;
        stm32_err_t s_err;

#ifdef STM32_COPROCESSOR_DEBUG
        AddLog(LOG_LEVEL_DEBUG, PSTR(STM_LOGNAME "STM32 erase memory"));
#endif  // STM32_COPROCESSOR_DEBUG

        stm32_erase_memory(stm, 0, STM32_MASS_ERASE);

        addr = stm->dev->fl_start;
        end = addr + size;
        while(addr < end && offset < size)
        {
            uint32_t left = end - addr;
            len   = sizeof(buffer) > left ? left : sizeof(buffer);
            len   = len > size - offset ? size - offset : len;

            if (len == 0)
            {
                break;
            }

            memcpy(buffer, p_st, sizeof(buffer));  // We need 4-byte bounadry flash access
            p_st += sizeof(buffer);

            s_err = stm32_write_memory(stm, addr, buffer, len);
            if (s_err != STM32_ERR_OK)
            {
                ret = false;
                break;
            }

            addr  += len;
            offset  += len;
        }
        stm32_close(stm);
    }
    return ret;
}

bool StmPresent(void) {
  return Stm.present;
}

uint32_t StmFlash(uint8_t* data, size_t size) {
// bool error = true;
#ifdef STM32_COPROCESSOR_DEBUG
  AddLog(LOG_LEVEL_INFO, PSTR(STM_LOGNAME "Updating firmware with %u bytes"), size);
#endif  // STM32_COPROCESSOR_DEBUG
    StmSerial->end();
    Serial.end(); // Close the default serial port (UART0) to avoid conflicts
    // if (StmSerial->begin(115200, SERIAL_8E1)) {
    Serial.begin(115200, SERIAL_8E1);
      // if (StmSerial->hardwareSerial())
      //     ClaimSerial();
#ifdef ESP32
      AddLog(LOG_LEVEL_DEBUG, PSTR(STM_LOGNAME "Flash via UART%d"), StmSerial->getUart());
#endif
      StmSerial->flush();   // clear the tx buffer
      StmResetToDFUMode();
      bool error = !StmUpdateFirmware(data, size);  // Allow flash access without ESP.flashRead
      // StmSerial->end();
      Serial.end();
      StmResetToAppMode();
      StmSerial->begin(115200, SERIAL_8N1);
    // }
  return error;
}

void StmResetToAppMode()
{
#ifdef STM32_COPROCESSOR_DEBUG
    AddLog(LOG_LEVEL_DEBUG, PSTR(STM_LOGNAME "Request co-processor reset in app mode"));
#endif  // STM32_COPROCESSOR_DEBUG

    pinMode(Pin(GPIO_STM32_RST_INV), OUTPUT);
    digitalWrite(Pin(GPIO_STM32_RST_INV), LOW);

    pinMode(Pin(GPIO_STM32_BOOT0), OUTPUT);
    digitalWrite(Pin(GPIO_STM32_BOOT0), LOW);

    delay(50);

    // clear in the receive buffer
    while (StmSerial->available())
        StmSerial->read();

    digitalWrite(Pin(GPIO_STM32_RST_INV), HIGH); // pull out of reset
    delay(50); // wait 50ms fot the co-processor to come online
}

bool StmInit(void)
{
#ifdef STM32_COPROCESSOR_DEBUG
    AddLog(LOG_LEVEL_INFO, PSTR(STM_LOGNAME "STM32 Driver Starting Tx %d Rx %d"), Pin(GPIO_STM32_TX), Pin(GPIO_STM32_RX));
#endif  // STM32_COPROCESSOR_DEBUG

    Stm.buffer = (uint8_t *)malloc(STM_BUFFER_SIZE);
    if (Stm.buffer != nullptr)
    {
        StmSerial = new TasmotaSerial(Pin(GPIO_STM32_RX), Pin(GPIO_STM32_TX), 0, 0, STM_BUFFER_SIZE); // Params: RX, TX, hardware_fallback (UART0,1,2), nwmode, buffer_size
        Serial.end(); // Close the default serial port (UART0) to avoid conflicts
        if (StmSerial->begin(115200))
        {
            if (StmSerial->hardwareSerial())
                ClaimSerial();

#ifdef ESP32
            AddLog(LOG_LEVEL_DEBUG, PSTR(STM_LOGNAME "Serial UART%d"), StmSerial->getUart());
#endif
            StmSerial->flush();
            StmResetToAppMode();
            // clear in the receive buffer
            while (StmSerial->available())
                StmSerial->read();
            delay(50); // wait 50ms for the co-processor to come online

            uint8_t startbyte = STM_START_BYTE;
            StmSerial->write(&startbyte, 1); // Send start byte to wake up the co-processor
            StmSerial->flush();

            return true;
        }
    }
    return false;
}

/*********************************************************************************************\
 * Internal Functions
\*********************************************************************************************/

bool StmSerialInput(void)
{
  if(StmSerial->available()){
    Stm.byte_counter = 0; // Buffer-Index zurücksetzen
    while (StmSerial->available() && Stm.byte_counter < STM_BUFFER_SIZE - 1)
    {
        yield();
        uint8_t serial_in_byte = StmSerial->read();
        Stm.buffer[Stm.byte_counter++] = serial_in_byte;
    }
    Stm.buffer[Stm.byte_counter] = '\0';
    ExecuteCommand((const char*)Stm.buffer, SRC_STM);
  }
  return true;
}

/*********************************************************************************************\
 * API Functions
\*********************************************************************************************/

bool StmModuleSelected(void) {
  if (PinUsed(GPIO_STM32_BOOT0) && PinUsed(GPIO_STM32_RST_INV)) {
    UpdateDevicesPresent(1);

    Stm.present = true;
  }
  return Stm.present;
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

// void STM_sendCmnd(uint8_t cmnd, uint8_t param) {
//   STMCommand.command = cmnd;
//   STMCommand.parameter = param;
//   char buffer[sizeof(STMCommand)];
//   memcpy(&buffer[0], &STMCommand, sizeof(STMCommand));

// #ifdef STM32_COPROCESSOR_DEBUG
// AddLog(LOG_LEVEL_INFO, PSTR(STM_LOGNAME "SendCmnd %*_H"), sizeof(buffer), (uint8_t*)&buffer);
// #endif
//   for (uint32_t ca = 0; ca < sizeof(buffer); ca++) {
//     StmSerial->write(buffer[ca]);
//     StmSerial->flush();
//   }
// }

#define D_PRFX_STM "Stm"
#define D_CMND_STM_RESET "Reset"
#define D_CMND_STM_SEND "Send"

const char kTasmotaStmCommands[] PROGMEM = D_PRFX_STM "|"
  D_CMND_STM_RESET "|" D_CMND_STM_SEND;

void (* const TasmotaStmCommand[])(void) PROGMEM = {
  &CmndStmReset, &CmndStmSend };

void CmndStmReset(void) {
  pinMode(Pin(GPIO_STM32_RST_INV), OUTPUT);
  digitalWrite(Pin(GPIO_STM32_RST_INV), LOW);
  delay(50);

  // clear in the receive buffer
  while (StmSerial->available())
      StmSerial->read();

  digitalWrite(Pin(GPIO_STM32_RST_INV), HIGH); // pull out of reset
  delay(50); // wait 50ms for the co-processor to come online
  ResponseCmndDone();
}

void CmndStmSend(void) {
  if (0 < XdrvMailbox.data_len) {
#ifdef STM32_COPROCESSOR_DEBUG
    char ascii_buf[XdrvMailbox.data_len + 1];
    memcpy(ascii_buf, XdrvMailbox.data, XdrvMailbox.data_len);
    ascii_buf[XdrvMailbox.data_len] = '\0';
    AddLog(LOG_LEVEL_INFO, PSTR(STM_LOGNAME "Sent data (%u bytes): %s"), XdrvMailbox.data_len, ascii_buf);
#endif
    uint8_t buf[XdrvMailbox.data_len + 1];
    memcpy(buf, XdrvMailbox.data, XdrvMailbox.data_len);
    buf[XdrvMailbox.data_len] = '\0';
    StmSerial->write(buf, XdrvMailbox.data_len + 1);
    StmSerial->flush();
  }
  ResponseCmndDone();
}

/*********************************************************************************************\
 * Driver Interface
\*********************************************************************************************/

bool Xdrv92(uint32_t function) {
  bool result = false;

  if (FUNC_MODULE_INIT == function) {
    result = StmModuleSelected();
  } else if (Stm.present) {
    switch (function) {
    case FUNC_EVERY_50_MSECOND:
      StmSerialInput(); // Process incoming data from the STM32 co-processor
      break;
      case FUNC_INIT:
        result = StmInit();
        break;
      case FUNC_COMMAND:
        result = DecodeCommand(kTasmotaStmCommands, TasmotaStmCommand);
        break;
      case FUNC_ACTIVE:
        result = true;
        break;
    }
  }

  return result;
}

#endif  // USE_STM32_COPROCESSOR
#if defined(BLE_SENSORS) && defined(ESP32)
//NimBLE-Arduino library used: https://github.com/h2zero/NimBLE-Arduino
//Custom firmware fot Xiaomi sensors: https://github.com/pvvx/ATC_MiThermometer
//Data parser: https://github.com/pvvx/ATC_MiThermometer/blob/master/esp32/Advertising/Advertising.ino


#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>

TaskHandle_t bleScanHandle = nullptr;
NimBLEScan* pBLEScan = NULL;
unsigned long *BLESensors_lastSuccesReading;
float *BLESensors_temp;
float *BLESensors_humidity;
float *BLESensors_pressure;
float *BLESensors_vbat;
bool ble_scan_mutex = false;
int ble_buffer_mutex = -1;
unsigned long BLESensors_readingInterval = 600; //10 min
int BLESensors_scanTime = 5;
int BLESensors_num_of_sensors = sizeof(BLE_sensors_macs)/sizeof(mac);

void BLESensors_freemem(void *ptr){
  if(ptr) {
    free(ptr);
    ptr = NULL;
  }
}

float BLESensors_readTemp(int sensorId){
  if(sensorId >= 0 && sensorId < BLESensors_num_of_sensors){
    while(ble_buffer_mutex == sensorId){
      delay(1);
    }
    return BLESensors_temp[sensorId];
  } else {
    return -273.14;
  }
}
float BLESensors_readHumidity(int sensorId){
  if(sensorId >= 0 && sensorId < BLESensors_num_of_sensors){
    while(ble_buffer_mutex == sensorId){
      delay(1);
    }
    return BLESensors_humidity[sensorId];
  } else {
    return -1;
  }
}
float BLESensors_readPressure(int sensorId){
  if(sensorId >= 0 && sensorId < BLESensors_num_of_sensors){
    while(ble_buffer_mutex == sensorId){
      delay(1);
    }
    return BLESensors_pressure[sensorId];
  } else {
    return -1;
  }
}
float BLESensors_readVbat(int sensorId){
  if(sensorId >= 0 && sensorId < BLESensors_num_of_sensors){
    while(ble_buffer_mutex == sensorId){
      delay(1);
    }
    return BLESensors_vbat[sensorId];
  } else {
    return -1;
  }
}
bool BLESensors_statusIsCorrect(int sensorId){
  if(sensorId >= 0 && sensorId < BLESensors_num_of_sensors){
    while(ble_buffer_mutex == sensorId){
      delay(1);
    }
    if(BLESensors_lastSuccesReading[sensorId] == 0) { return false;}
    unsigned long now = millis();
    if(BLESensors_lastSuccesReading[sensorId] > now) {
      unsigned long diff = (unsigned long)(-1) - BLESensors_lastSuccesReading[sensorId] + now;
      if(diff > BLESensors_readingInterval * 2500) { //25 min
        return false;
      }
    } else {
      unsigned long diff = now - BLESensors_lastSuccesReading[sensorId];
      if(diff > BLESensors_readingInterval * 2500) { //25 min
        return false;
      }
    }
    return true;
  } else {
    return false;
  }
}

void BLESensors_printBuffer(uint8_t* buf, int len) {
  for (int i = 0; i < len; i++) {
    printFmtToDebug(PSTR("%02x"), buf[i]);
  }
  printToDebug(PSTR("\r\n"));
}

class QueryBLESensors: public NimBLEAdvertisedDeviceCallbacks {
  public:
    QueryBLESensors(){
    }

    ~QueryBLESensors(){
    }

    void parse_value(uint8_t* buf, int len, int sensorID) {
      int16_t x = buf[3];
      if (buf[2] > 1)
        x |=  buf[4] << 8;
      switch (buf[0]) {
        case 0x0D:
          if (buf[2] && len > 6) {
            BLESensors_temp[sensorID] = x / 10.0;
            x =  buf[5] | (buf[6] << 8);
            BLESensors_humidity[sensorID]= x / 10.0;
            printFmtToDebug(PSTR("Temp: %.1f°, Humidity: %.1f %%\r\n"), BLESensors_temp[sensorID], BLESensors_humidity[sensorID]);
          }
          break;
        case 0x04: {
            BLESensors_temp[sensorID] = x / 10.0;
            printFmtToDebug(PSTR("Temp: %.1f°\r\n"), BLESensors_temp[sensorID]);
          }
          break;
        case 0x06: {
            BLESensors_humidity[sensorID] = x / 10.0;
            printFmtToDebug(PSTR("Humidity: %.1f%%\r\n"), BLESensors_humidity[sensorID]);
          }
          break;
        case 0x0A: {
            printFmtToDebug(PSTR("Battery: %d%%"), x);
            if (len > 5 && buf[4] == 2) {
              BLESensors_vbat[sensorID] = (buf[5] | (buf[6] << 8)) / 1000;
              printFmtToDebug(PSTR(", %.3f V"), BLESensors_vbat[sensorID]);
            }
            printToDebug(PSTR("\r\n"));
          }
          break;
        default:
          printFmtToDebug(PSTR("Type: 0x%02x "), buf[0]);
          BLESensors_printBuffer(buf, len);
          break;
      }
    }

    uint8_t* findServiceData(uint8_t* data, size_t length, uint8_t* foundBlockLength) {
      uint8_t* rightBorder = data + length;
      while (data < rightBorder) {
        uint8_t blockLength = *data + 1;
        //Serial.printf("blockLength: 0x%02x\n",blockLength);
        if (blockLength < 5) {
          data += blockLength;
          continue;
        }
        uint8_t blockType = *(data + 1);
        uint16_t serviceType = *(uint16_t*)(data + 2);
        //Serial.printf("blockType: 0x%02x, 0x%04x\n", blockType, serviceType);
        if (blockType == 0x16) { // https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/
          // Serial.printf("blockType: 0x%02x, 0x%04x\n", blockType, serviceType);
          /* 16-bit UUID for Members 0xFE95 Xiaomi Inc. https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf */
          if (serviceType == 0xfe95 || serviceType == 0x181a) { // mi or custom service
            //Serial.printf("blockLength: 0x%02x\n",blockLength);
            //Serial.printf("blockType: 0x%02x, 0x%04x\n", blockType, serviceType);
            *foundBlockLength = blockLength;
            return data;
          }
        }
        data += blockLength;
      }
      return nullptr;
    }

    int MAC_in_list(NimBLEAddress *addr){
      byte mac[6];
      memcpy(mac, addr->getNative(), 6);
      for(int i = 0; i < BLESensors_num_of_sensors; i++){
//        BLESensors_printBuffer((uint8_t *)mac->getNative(), 6);
//        BLESensors_printBuffer((uint8_t *)BLE_sensors_macs + i * 6, 6);
        bool same = true;
        for(uint8_t j = 0; j < 6; j++){
          //byte order in addr->getNative is inverted
          if(mac[5 - j] != BLE_sensors_macs[i][j]) {
            same = false;
            break;
          }
        }
        if(same) return i;
        /*if(memcmp(addr->getNative(), BLE_sensors_macs + i * 6, 6) == 0){
          return i;
        }*/
      }
      return -1;
    }

    void onResult(NimBLEAdvertisedDevice *advertisedDevice) {
      uint8_t* payload = advertisedDevice->getPayload();
      size_t payloadLength = advertisedDevice->getPayloadLength();
      //esp_bd_addr_t is uint8_t mac[6];
      NimBLEAddress *deviceAddress = new NimBLEAddress(advertisedDevice->getAddress());
      uint8_t serviceDataLength = 0;
      uint8_t* serviceData = findServiceData(payload, payloadLength, &serviceDataLength);
      //printFmtToDebug(PSTR("Found device %s<BR>\r\n"), advertisedDevice->toString().c_str());
      //printFmtToDebug(PSTR("serviceDataLength %d<BR>\r\n"), serviceDataLength);
      if (serviceData == nullptr || serviceDataLength < 15) {
        delete(deviceAddress);
        return;
      }
      uint16_t serviceType = *(uint16_t*)(serviceData + 2);
      int sensorID = MAC_in_list(deviceAddress);
      printFmtToDebug(PSTR("Found device "));
      BLESensors_printBuffer((uint8_t *)deviceAddress->getNative(), 6);
      delete(deviceAddress);
      //printFmtToDebug(PSTR("sensorID %d<BR>\r\n"), sensorID);
      if(sensorID == -1) return;
      ble_buffer_mutex = sensorID;
      BLESensors_lastSuccesReading[sensorID] = millis();
      ble_buffer_mutex = -1;
      printFmtToDebug(PSTR("Found service '%04x' data len: %d, "), serviceType, serviceDataLength);
      BLESensors_printBuffer(serviceData, serviceDataLength);
      if (serviceType == 0xfe95) {
        if ((serviceData[5] & 0x08) == 0) { // not encrypted
          serviceDataLength -= 15;
          payload = &serviceData[15];
          ble_buffer_mutex = sensorID;
          while (serviceDataLength > 3) {
            parse_value(payload, serviceDataLength, sensorID);
            serviceDataLength -= payload[2] + 3;
            payload += payload[2] + 3;
          }
          ble_buffer_mutex = -1;
          printFmtToDebug(PSTR("count: %d\r\n"), serviceData[8]);
        } else {
          if (serviceDataLength > 19) { // aes-ccm  bindkey
            // https://github.com/ahpohl/xiaomi_lywsd03mmc
            // https://github.com/Magalex2x14/LYWSD03MMC-info
            printFmtToDebug(PSTR("Crypted data[%d]! "), serviceDataLength - 15);
          }
          printFmtToDebug(PSTR("count: %d\r\n"), serviceData[8]);
        }
      } else { // serviceType == 0x181a
        ble_buffer_mutex = sensorID;
        if(serviceDataLength > 18) { // custom format
          BLESensors_temp[sensorID] = *(int16_t*)(serviceData + 10) / 100.0;
          BLESensors_humidity[sensorID] = *(uint16_t*)(serviceData + 12) / 100.0;
          BLESensors_vbat[sensorID] = *(uint16_t*)(serviceData + 14) /1000;
          printFmtToDebug(PSTR("Temp: %.2f°, Humidity: %.2f%%, Vbatt: %.3f, Battery: %d%%, flg: 0x%02x, cout: %d\r\n"), BLESensors_temp[sensorID], BLESensors_humidity[sensorID], BLESensors_vbat[sensorID], serviceData[16], serviceData[18], serviceData[17]);
        } else if(serviceDataLength == 17) { // format atc1441
//          Serial.printf("MAC: "); printBuffer(serviceData + 4, 6);
          int16_t x = (serviceData[10]<<8) | serviceData[11];
          BLESensors_temp[sensorID] = x / 10.0;
          x = (serviceData[14]<<8) | serviceData[15];
          BLESensors_vbat[sensorID] = x / 1000;
          BLESensors_humidity[sensorID] = serviceData[12];
          printFmtToDebug(PSTR("Temp: %.1f°, Humidity: %f%%, Vbatt: %.3f, Battery: %d%%, cout: %d\r\n"), BLESensors_temp[sensorID], BLESensors_humidity[sensorID], BLESensors_vbat[sensorID], serviceData[13], serviceData[16]);
        }
      ble_buffer_mutex = -1;
      }
    }
};

QueryBLESensors *queryBLESensorsResult = NULL;

class ListBLEDevicesAround: public NimBLEAdvertisedDeviceCallbacks {
  private:
    uint8_t *MACsWhichWasAlreadyFounded = NULL;
    uint16_t devices = 0;
    static const uint16_t max_devices = 100;

  public:
    ListBLEDevicesAround(){
      MACsWhichWasAlreadyFounded = (uint8_t *)malloc(sizeof(uint8_t) * 6 * max_devices); //Table for duplicate checking.
      reset();
    }

    ~ListBLEDevicesAround(){
      if(MACsWhichWasAlreadyFounded) free(MACsWhichWasAlreadyFounded);
    }

    void reset(){
      devices = 0;
      memset(MACsWhichWasAlreadyFounded, 0, sizeof(uint8_t) * 6 * max_devices);
    }

    int DeviceIsAlreadyAdvertised(NimBLEAddress *mac){
      uint8_t _mac[6];
      memcpy(_mac, mac->getNative(), 6);
      for(int i = 0; i < max_devices; i++){
        if(memcmp(_mac, MACsWhichWasAlreadyFounded + i * 6, 6) == 0){
          return i;
        }
      }
      if(devices >= max_devices) return -2;
      memcpy(MACsWhichWasAlreadyFounded + devices * 6, _mac, 6);
      devices++;
      return -1;
    }

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
      if(!MACsWhichWasAlreadyFounded) return;
      NimBLEAddress *deviceAddress = new NimBLEAddress(advertisedDevice->getAddress());
      int result = DeviceIsAlreadyAdvertised(deviceAddress);
      delete(deviceAddress);
      if(result < 0) {
        printFmtToWebClient(PSTR("#%d: %s<BR>\r\n"), devices, advertisedDevice->toString().c_str());
      }
    }
};

static void scanCompleteCB(NimBLEScanResults scanResults) {
  if(bleScanHandle != nullptr) {
    vTaskDelete( bleScanHandle );
    bleScanHandle = nullptr;
  }
  printlnToDebug(PSTR("BLE background scan complete!"));
  pBLEScan->stop();
  pBLEScan->clearResults();
  ble_scan_mutex = false;
}

void BLESensors_taskScanBleDevices(void* pvParameters) {
  printlnToDebug(PSTR("BLE background scan start!"));
        pBLEScan->start(0, scanCompleteCB, true);
        while(true){
          printlnToDebug(PSTR("BLE scan tick"));
          vTaskDelay(30000 / portTICK_RATE_MS);
          printlnToDebug(PSTR("BLE scan tack"));
          pBLEScan->clearResults();
        }


  /*              while(true){
                  pBLEScan->start(5, true);
                  pBLEScan->stop();
                  pBLEScan->clearResults();
                  printlnToDebug(PSTR("tick"));
                  vTaskDelay(60000 / portTICK_RATE_MS);
                  printlnToDebug(PSTR("tack"));
                }
                */
}

void BLESensors_scan_sensors(bool async, bool activeScan){
  if(ble_scan_mutex) {
    pBLEScan->stop();
    pBLEScan->clearResults();
  }
  while(ble_scan_mutex){
    delay(1);
  }
  ble_scan_mutex = true;
  if(!queryBLESensorsResult) queryBLESensorsResult = new QueryBLESensors();
  pBLEScan->setAdvertisedDeviceCallbacks(queryBLESensorsResult, true);
  //pBLEScan->setInterval(625); // default 100
  //pBLEScan->setWindow(625);  // default 100, less or equal setInterval value
  pBLEScan->setActiveScan(activeScan);
  if(async) {
    xTaskCreate(BLESensors_taskScanBleDevices, "ble-scan", 2048, nullptr, tskIDLE_PRIORITY, &bleScanHandle);
  } else {
    pBLEScan->start(5, true);
//    xTaskCreate(BLESensors_taskScanBleDevices, "ble-scan", 2048, nullptr, tskIDLE_PRIORITY, &bleScanHandle);
    pBLEScan->stop();
    pBLEScan->clearResults();
    ble_scan_mutex = false;
  }
}

int num_of_queried_devices(){
  int devices = 0;
  for(int i = 0; i < sizeof(BLE_sensors_macs)/sizeof(mac); i++) {
    for(int j = 0; j < sizeof(mac); j++){
      if(BLE_sensors_macs[i][j]){
        devices++;
        break;
      }
    }
  }
  return devices;
}

void printListBLEDevicesAround(){
  if(ble_scan_mutex) {
    pBLEScan->stop();
    pBLEScan->clearResults();
  }
  while(ble_scan_mutex){
    delay(1);
  }
  ble_scan_mutex = true;
  pBLEScan->setActiveScan(true);
  ListBLEDevicesAround *cb = new ListBLEDevicesAround();
  pBLEScan->setAdvertisedDeviceCallbacks(cb, true);
  pBLEScan->start(5, false);
  pBLEScan->stop();
  pBLEScan->clearResults();
  delete(cb);
  ble_scan_mutex = false;
  BLESensors_scan_sensors(true, false);
}

void BLESensors_init() {
  int devices = num_of_queried_devices();
  NimBLEDevice::init(PSTR("BSB-LAN"));
  BLESensors_lastSuccesReading = (unsigned long *)malloc(sizeof(unsigned long) * devices);
  memset(BLESensors_lastSuccesReading, 0, sizeof(unsigned long) * devices);
  BLESensors_temp = (float *)malloc(sizeof(float) * devices);
  BLESensors_humidity = (float *)malloc(sizeof(float) * devices);
  BLESensors_pressure = (float *)malloc(sizeof(float) * devices);
  BLESensors_vbat = (float *)malloc(sizeof(uint16_t) * devices);
  BLESensors_num_of_sensors = devices;
  pBLEScan = NimBLEDevice::getScan();
}

void BLESensors_destroy(){
  BLESensors_num_of_sensors = 0;
  if(ble_scan_mutex) {
    pBLEScan->stop();
    pBLEScan->clearResults();
  }
  if(queryBLESensorsResult) {
    delete(queryBLESensorsResult);
    queryBLESensorsResult = NULL;
  }
  BLESensors_freemem(BLESensors_lastSuccesReading);
  BLESensors_freemem(BLESensors_temp);
  BLESensors_freemem(BLESensors_humidity);
  BLESensors_freemem(BLESensors_pressure);
  BLESensors_freemem(BLESensors_vbat);
}
#endif

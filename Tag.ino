#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include "RangeFilter.h"
#include "Trilateration.h"
#include <map>

// Map to store filters
std::map<uint16_t, RangeFilter*> deviceFilters;

// Trilateration Solver
Trilateration trilat;

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

void setup()
{
  Serial.begin(115200);
  delay(1000);

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

// Start as Tag
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  
  // Initialize Anchors with Coordinates (METERS)
  // Practical "Room Setup" (2.8m x 2.9m)
  // Updated to match your Measured Values
  trilat.addAnchor(0x01A0, 0.0, 0.0);  // Anchor 1: Seen as "1A0"
  trilat.addAnchor(0x02A0, 2.8, 0.0);  // Anchor 2: Seen as "2A0"
  trilat.addAnchor(0x03A0, 0.0, 2.9);  // Anchor 3: Seen as "3A0"
  
  // NOTE: You must update these IDs to match your actual Anchor Short Addresses (HEX)
  Serial.println("Init Trilateration... Please configure Anchor Coordinates in setup()!");
}

void loop() {
  DW1000Ranging.loop();
}

void newRange()
{
  uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();
  float rawRange = DW1000Ranging.getDistantDevice()->getRange();

  if (deviceFilters.find(addr) == deviceFilters.end()) {
    deviceFilters[addr] = new RangeFilter(10); 
  }
  
  deviceFilters[addr]->addValue(rawRange);
  float filteredRange = deviceFilters[addr]->getMedian();

  Serial.print(addr, HEX);
  Serial.print(",");
  Serial.println(filteredRange);
  
  // Update Trilateration Engine
  trilat.updateDistance(addr, filteredRange);
  
  // Try to compute position
  Point pos;
  if (trilat.calculatePosition(pos)) {
    Serial.print("POS,");
    Serial.print(pos.x);
    Serial.print(",");
    Serial.println(pos.y);
  }
}

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
  
  uint16_t addr = device->getShortAddress();
  if (deviceFilters.find(addr) != deviceFilters.end()) {
    delete deviceFilters[addr];
    deviceFilters.erase(addr);
  }
}

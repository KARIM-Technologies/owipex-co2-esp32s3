#include <EEPROM.h>

#define EEPROM_SIZE 512 // Stellen Sie sicher, dass diese Größe Ihrer tatsächlichen EEPROM-Größe entspricht

void setup() {
  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);

  // Lösche den gesamten EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit(); // Übertrage die Änderungen auf den EEPROM

  Serial.println("EEPROM wurde komplett gelöscht.");
}

void loop() {
  // Keine Operationen im Loop.
}

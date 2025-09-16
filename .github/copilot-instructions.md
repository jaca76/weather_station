### Podsumowanie projektu

Ten projekt to stacja pogodowa oparta na płycie TTGO LoRa32 (ESP32), która odczytuje dane z czujnika BME280 (temperatura, wilgotność, ciśnienie), wyświetla je na wbudowanym ekranie OLED i wysyła za pośrednictwem komunikacji LoRa. Projekt jest skonfigurowany przy użyciu PlatformIO.

### Kluczowe technologie i biblioteki

- **Framework**: Arduino
- **Platforma**: Espressif 32 (dla płyty `ttgo-lora32-v21`)
- **Główne zależności** (z `platformio.ini`):
  - `sandeepmistry/LoRa`: Do komunikacji radiowej LoRa.
  - `adafruit/Adafruit BME280 Library`: Do interakcji z czujnikiem pogody.
  - `olikraus/U8g2`: Do sterowania wyświetlaczem OLED.
  - `bblanchon/ArduinoJson`: Do serializacji danych czujników do formatu JSON przed wysłaniem.

### Architektura i przepływ danych

1.  **Inicjalizacja**: W funkcji `setup()` inicjowane są: komunikacja szeregowa, wyświetlacz OLED, czujnik BME280 i moduł LoRa.
2.  **Pętla główna (`loop()`)**:
    - Odczytywane są wartości z czujnika BME280.
    - Dane są wyświetlane na ekranie OLED.
    - Dane są pakowane do obiektu JSON.
    - Zserializowany ciąg JSON jest wysyłany jako pakiet LoRa.
    - Urządzenie przechodzi w stan uśpienia na określony czas, aby oszczędzać energię.

### Konwencje programistyczne

- Główna logika aplikacji znajduje się w pliku `src/LORA_SNDR.ino`.
- Konfiguracja sprzętu (piny, częstotliwości) jest zdefiniowana na początku pliku `.ino` za pomocą stałych (`#define`).
- Dane czujników są strukturyzowane w formacie JSON w celu transmisji.

### Przepływy pracy programisty

- **Budowanie i wgrywanie**: Użyj standardowych poleceń PlatformIO:
  - `pio run`: Aby zbudować projekt.
  - `pio run -t upload`: Aby zbudować i wgrać oprogramowanie na podłączone urządzenie.
  - `pio run -t monitor`: Aby otworzyć monitor szeregowy i wyświetlić logi (prędkość transmisji: 115200).
- **Zarządzanie zależnościami**: Biblioteki są zarządzane w pliku `platformio.ini` w sekcji `lib_deps`. Użyj `pio lib install` w razie potrzeby.

# Mostek H Modbus TCP
WT32-ETH01 + BTS7960B + TXS0108E. Wersja 1.1.

## Wgrywanie
Arduino IDE: ESP32 Dev Module, esp32 by Espressif Systems 3.x,
ArduinoJson by Benoit Blanchon 7.x.
Partycje: Default 4MB with spiffs (uklad z OTA; nie Huge APP / No OTA).
Pliki WT32_HBridge_Modbus.ino i secrets.h musza byc w folderze
WT32_HBridge_Modbus. Pierwsze wgranie przez adapter USB-UART.
Kolejne: Szkic -> Eksportuj skompilowany program; wybierz plik aplikacji .ino.bin,
nie merged.bin, bootloader ani partitions.bin.

## secrets.h
Mozesz skopiowac swoj secrets.h z odbiornika suszarni.
Obslugiwane makra:
DRYER_BOOTSTRAP_WIFI, DRYER_BOOTSTRAP_WIFI_PASSWORD,
DRYER_BOOTSTRAP_OTA_PASSWORD, DRYER_WIFI_NETWORKS.
DRYER_WIFI_NETWORKS to tekst JSON: tablica obiektow ssid/password.
W paczce secrets.h zawiera TYLKO przykladowe dane.
Zmien haslo CHANGE_ME. Puste haslo lub CHANGE_ME blokuje OTA.
Do GitHuba dodawaj tylko secrets.example.h. secrets.h jest w .gitignore.
Format makr odtworzono z wczesniejszych ustalen projektu suszarni;
nie porownano go z Twoim rzeczywistym plikiem.

## Siec i lokalne OTA
Ethernet: 192.168.1.13, maska255.255.255.0, brama192.168.1.1.
Wi-Fi: kolejno do 12 roznych wpisow; kazda proba trwa okolo12s;
bootstrap dopisywany jako rezerwa. Adres Wi-Fi z DHCP.
Polaczenia Wi-Fi sa ponawiane tylko przy zatrzymanym napedzie.
Monitor szeregowy115200 pokazuje adresy. Nie wypisuje hasel.
OTA: http://192.168.1.13/update albo http://ADRES_WIFI/update.
Login admin, haslo DRYER_BOOTSTRAP_OTA_PASSWORD.
Internet nie jest potrzebny; program niczego z niego nie pobiera.
Nie przekierowuj portow z internetu.
Przed otwarciem strony wyslij STOP. Podczas ruchu serwer HTTP czeka.
Start przesylania odlacza klienta Modbus i blokuje naped.
Po bledzie aktualizacji, przerwaniu polaczenia lub restarcie wymagany nowy STOP.
Aktualizacja nie powinna byc wykonywana w trakcie cyklu suszarni.

## Modbus TCP
Port502, UnitID1; jeden klient. Funkcje03,06,16.
Adresy protokolu liczone od zera (40001 odpowiada0).
0: kierunek0 STOP,1 prawo,2 lewo (zapis/odczyt).
1: PWM0..1000 = 0..100% (zapis/odczyt).
2: aktualne PWM (odczyt).
3: aktualnie sterowany kierunek (odczyt).
4: odblokowany0/1 (odczyt).
5: wersja110 (odczyt).
Po starcie najpierw zapis0 do rejestru0.
Nastepnie zapisuj [kierunek,PWM] funkcja16 do0..1 co500ms.
Brak zapisu kierunku przez1500ms: natychmiastowe wylaczenie i blokada.
Odczyty i zapis samego PWM nie odnawiaja czasu.
STOP jest natychmiastowy. Rozruch PWM narasta okolo1s od0 do100%.
Zmiana kierunku: zejscie PWM do0 i minimum700ms przerwy.
To NIE jest pomiar predkosci ani potwierdzenie mechanicznego zatrzymania.

## Polaczenia
WT32 GPIO32/CFG -> TXS A1; B1 -> RPWM.
WT32 GPIO33/485_EN -> TXS A2; B2 -> LPWM.
WT32 GPIO14 -> TXS A3; B3 -> R_EN i L_EN razem.
WT32 GPIO4 -> OE, rezystor10k OE doGND.
TXS VA=3V3 zWT32, VB=5V, GND wspolna.
Mostek VCC=5V. WT32 pin5V=5V z przetwornicy12->5V.
Mostek B+=12V przez bezpiecznik dobrany do przewodow i napedu,
B-=minus12V; W+/W-=silnik; R_IS/L_IS niepodlaczone.
Prad silnika nie moze przeplywac przez przewody masy elektroniki.

## Status uruchomienia
Wersja do prob stolowych, NIE potwierdzona na sprzecie.
Nie potwierdzono kompilacji: lokalny kompilator zostal zablokowany
przez srodowisko wykonawcze (winapi623).
OE w stanie LOW ustawia TXS w wysoka impedancje, nie wymusza zera.
Przed podaniem12V na B+ nalezy zapewnic sprzetowe default-OFF na R_EN/L_EN
i sprawdzic start, reset i utrate zasilania ESP.
Sam rezystor10k na OE nie zapewnia wylaczenia mostka.
Brak programowego pomiaru/zabezpieczenia pradu i temperatury.
40A rozruchowe silnika nie oznacza potwierdzonej obciazalnosci modulu.


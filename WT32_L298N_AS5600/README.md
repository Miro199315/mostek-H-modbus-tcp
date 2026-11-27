# WT32-ETH01 — klapka 0–10, L298N, AS5600, rynna
Firmware v4.0. Ethernet 192.168.1.13/24, brama 192.168.1.1.
Wi-Fi DHCP. Modbus TCP port 502, Unit 1. WWW i OTA port 80.

## Podlaczenie — bez zmian
Numery GPIO, nie numery kolejnych pinow zlacza.

| Element | WT32-ETH01 |
| --- | --- |
| L298N IN1, klapka | GPIO32 / CFG |
| L298N IN2, klapka | GPIO33 / 485_EN |
| L298N IN3, rynna | GPIO14 |
| L298N IN4, rynna | GPIO4 |
| AS5600 SDA | GPIO17 / TXD po lewej stronie |
| AS5600 SCL | GPIO5 / RXD po lewej stronie |
| AS5600 VCC | 3V3, modul zgodny z 3,3 V |
| AS5600 DIR | GND |
| Wszystkie masy | Wspolne GND ESP, enkodera, L298N i zasilacza |
| L298N OUT1/2 | Silnik klapki |
| L298N OUT3/4 | Silownik rynny |
| L298N ENA i ENB | Zworki / stan wysoki |

WT32 zasil stabilizowanym 5 V. Stosuj krotkie I2C, z dala od przewodow silnika.
IN1..IN4: po 10 kohm do GND przy sterowniku. Przy 24 V na typowym L298N
zdejmij zworke regulatora 5V-EN i osobno zasil logike 5 V zgodnie z modulem.
5V-EN i ENA/ENB to rozne zworki.
Programowanie: TX0 GPIO1 / RX0 GPIO3, poziomy 3,3 V. GPIO0 do GND tylko na
czas wejscia w tryb wgrywania; odlacz przed normalnym startem Ethernetu.

## Nowa skala i kalibracja
Na stronie nie zadaje sie juz stopni. Skala wynosi 0–10, nastawy co 0,1.
0 i 10 to zapisane przez uzytkownika pozycje klapki. 5 jest dokladnie
w polowie przejechanego zakresu; np. 2,5 to 25% zakresu.
Nie jest to kalibracja przeplywu powietrza — przeplyw moze byc nieliniowy.

1. Przytrzymuj LEWO lub PRAWO, ustaw pierwszy koniec i pusc przycisk.
2. Na postoju nacisnij „Zapisz aktualna pozycje jako 0”.
3. Podjedz recznie do drugiego konca. Podczas przejazdu enkoder musi dzialac.
4. Pusc przycisk i nacisnij „Zapisz aktualna pozycje jako 10”.
5. Zadaj np. 2,5, 5 lub 7,8 i nacisnij „Ustaw pozycje”.

Zakres i kierunek sa zapisane razem w NVS. Po restarcie pozycja jest odczytywana
z absolutnego AS5600 bez bazowania. Obslugiwany jest zakres od 1 do 350 stopni,
rowniez przez przejscie surowego odczytu 4095/0 i dluzszy niz pol obrotu.
Cel jest wybierany wewnatrz zapisanego luku, bez skrotu dookola.
Zapis nowego 0 uniewaznia stary punkt 10. Nalezy zapisac oba punkty.
Przerwa w odczycie enkodera lub restart pomiedzy zapisem 0 i 10 wymaga
ponownego rozpoczecia od 0. Blad zapisu NVS nie podmienia aktywnej kalibracji.

Po aktualizacji z v3.x stare ZERO nie jest automatycznie skala. Wymagane
jest jednorazowe zapisanie 0 i 10. Ustawienia Wi-Fi i haslo sa bez zmian.

## Ruch i testy
Klapka jedzie od razu na 100% mocy, zatrzymuje sie przy celu i wykonuje
krotkie korekty pelna moca. Nastawa ma krok 0,1 jednostki skali; mechaniczna
dokladnosc zalezy od enkodera, luzow i dobiegu napedu. Wewnetrzna tolerancja
odczytu pozostaje 0,1 stopnia, nie 0,1 jednostki skali.
STOP zatrzymuje oba napedy. Bledy enkodera, brak postepu i limit czasu
zatrzymuja pozycjonowanie. Ruch sieciowy wymaga odswiezania polecen.

Reczne LEWO/PRAWO jest trybem ustawiania, omija skale i dziala nawet bez
enkodera. Brak kontroli krancowek. Puszczenie przycisku lub brak jego
odnowienia przez 500 ms zatrzymuje test; limit nacisniecia 10 s.
Zmiana kierunku recznego wymaga 700 ms przerwy.
Zapis punktow skali wymaga postoju obu napedow i prawidlowego AS5600.

W sekcji zaawansowanej zachowano testy impulsow LEWO/PRAWO z v3.3.
To osobna kalibracja dobiegu: 3 impulsy po 150 ms, przerwy 700 ms,
wymagane minimum 8 stopni wolnego ruchu. Nie zmienia punktow skali.

## Arduino i OTA
Zachowaj pliki .ino, motion.h i secrets.example.h w jednym folderze szkicu.
Skopiuj swoj secrets.h. Nie publikuj hasel.
Makra: DRYER_BOOTSTRAP_WIFI, DRYER_BOOTSTRAP_WIFI_PASSWORD,
DRYER_BOOTSTRAP_OTA_PASSWORD, DRYER_WIFI_NETWORKS (JSON ssid/password).
Do 6 sieci Wi-Fi. Login WWW: admin; haslo: DRYER_BOOTSTRAP_OTA_PASSWORD.
Puste haslo lub CHANGE_ME wylacza WWW i OTA.

Arduino-ESP32 3.3.0, ArduinoJson 7.4.2.
ESP32 Dev Module, Flash 4 MB, PSRAM Disabled.
Partition Scheme: Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS), min_spiffs.
Szkic > Eksportuj skompilowany program, a nastepnie otworz
http://192.168.1.13/update i wgraj WT32_L298N_AS5600.ino.bin.
Nie wgrywaj merged.bin, bootloader.bin ani partitions.bin przez OTA.
Po restarcie http://192.168.1.13/ i Ctrl+F5; naglowek ma v4.0.
ArduinoOTA rowniez dostepne, host suszarnia-klapka, to samo haslo.
Pierwsze wgranie i zmiana partycji przez USB-UART.
OTA zatrzymuje oba napedy; po restarcie nie wznawia ruchu.
Przerwany zapis zachowuje stara aplikacje, jesli nie zatwierdzono nowej partycji.
Nie ma automatycznej oceny nowego programu ani gwarancji rollback po zawieszeniu.

## Modbus TCP v4.0
Adresy ponizej liczone od zera. FC03/04/06/16, maksymalnie 3 klientow.
UWAGA: rejestry klapki 6 i 8 maja od v4.0 jednostki nowej skali,
a nie stopnie. Rejestry rynny pozostaja bez zmian.

| Adres | Dostep | Znaczenie |
| --- | --- | --- |
| 0 | RW | Rynna: 0 STOP, 1 gora, 2 dol |
| 1 | RW | PWM rynny 0..1000 |
| 2 | R | Aktualny PWM rynny |
| 3 | R | Aktualny kierunek rynny |
| 4 | R | Rynna odblokowana po STOP |
| 5 | R | Wersja 400 |
| 6 | RW | Cel klapki 0..100: 25 oznacza 2,5 |
| 7 | W | 0 STOP klapki, 1 jedz do rejestru 6, 2 zapisz 0, 3 zapisz stare tuning, 4 zapisz 10 |
| 8 | R | Pozycja skali x10, ograniczona do 0..100; 65535 gdy brak kalibracji/odczytu |
| 9 | R | Flagi: PWM gotowe 1, enkoder OK 2, oba konce zapisane 4, ruch 8, cel osiagniety 16, blad 32, OTA 64 |
| 10 | RW | Stare SPEED x10, 10..120, bez wplywu na klapke przy pelnej mocy |
| 11 | RW | Stare ACCEL x10, 10..1200, bez wplywu na klapke przy pelnej mocy |
| 12 | R | Blad: 0 brak, 1 ruch/czujnik, 2 komunikacja, 3 zapis NVS |
| 13 | RW | Heartbeat klapki, zapis w tej samej sesji TCP co GO |
| 14 | W | 1 STOP obu napedow i rozbrojenie rynny |
| 15 | R | Surowy AS5600 0..4095 |

Przyklad po kalibracji: FC16 adres6 wartosci [75,1] ustawia klapke na 7,5.
Zapisuj rejestr13 co 500 ms w tej samej sesji az do konca ruchu.
Odczyt statusu nie odnawia heartbeat. Limit 1500 ms; zamkniecie sesji lub
utrata sieci zatrzymuje nalezacy do niej ruch. WWW odnawia swoj ruch co 400 ms.
Kalibracja obu koncow i zadawanie pozycji sa tez dostepne z WWW i USB.

Rynna: zgodnosc ChuteLift suszarni zachowana. FC16 adres0 wartosci [0,0]
odblokowuje; potem co 100 ms [kierunek,PWM]. Zamkniecie sesji zatrzymuje
rynne. Przerwa kierunkow minimum 700 ms. Odbiornik suszarni juz rampuje PWM.
Automatyczne przestawianie klapki wedlug etapu palnika wymaga wywolan
nowych rejestrow z odbiornika; nie dodano tego w tej aktualizacji.

## USB i diagnostyka
115200, Nowa linia: ZERO zapis0, TEN zapis10, liczba np. 2,5 albo 7.8,
STOP, STATUS. Skala z USB rowniez wymaga obu punktow.
WWW pokazuje pozycje i cel w nowej skali, stan enkodera, kalibracje,
polecenie PWM i szczegolowy opis bledow. /api/status zachowuje techniczne
pola angle/target dla diagnostyki; nowe scale/scaleTarget sa jednostkami skali.
Polecenie WWW move przyjmuje pole scale, nie stare angle.
STOP zachowuje ostatni blad; nowe przyjete polecenie lub poprawny zapis
punktu skali go kasuje. Blad nie jest zapisywany przez restart.

# WT32-ETH01 — L298N, klapka AS5600 i rynna

Firmware v3.0 dla WT32-ETH01 z LAN8720. Klapka jest sterowana pierwszym kanalem L298N, a silownik rynny drugim. AS5600 mierzy absolutny kat klapki. IP Ethernet 192.168.1.13, maska 255.255.255.0, brama 192.168.1.1. Wi-Fi pobiera osobny adres przez DHCP.

## Podlaczenie

Numery oznaczaja GPIO, a nie numery fizycznych pinow zlacza.

| Element | WT32-ETH01 |
| --- | --- |
| L298N IN1, klapka | GPIO32, napis CFG |
| L298N IN2, klapka | GPIO33, napis 485_EN |
| L298N IN3, rynna | GPIO14 |
| L298N IN4, rynna | GPIO4 |
| AS5600 SDA | GPIO17, napis TXD na glownym zlaczu |
| AS5600 SCL | GPIO5, napis RXD na glownym zlaczu |
| AS5600 VCC, modul zgodny z 3,3 V | 3V3 |
| AS5600 DIR, jezeli dostepny | GND |
| AS5600 GND, L298N GND, minus zasilacza | Wspolna masa GND |
| L298N OUT1/OUT2 | Silnik klapki |
| L298N OUT3/OUT4 | Silownik rynny |
| L298N ENA/ENB | Zworki zalozone, poziom wysoki |

Na IN1..IN4 zastosuj po 10 kohm do GND po stronie L298N. WT32 zasil stabilizowanym 5 V. Oba silniki L298N korzystaja ze wspolnego zasilania mocy i musza byc do niego dobrane. Przy 24 V na typowym module zdejmij zworke regulatora 5V-EN i zasil logike osobnym stabilizowanym 5 V zgodnie z oznaczeniami modulu. ENA/ENB to oddzielne zworki. L298N ma limit 2 A na kanal, a rozruch i zablokowanie moga wymagac wiekszego pradu. Testy obciazenia sa konieczne do potwierdzenia doboru.

Do USB-UART uzyj TX0 GPIO1 i RX0 GPIO3 na zlaczu programowania, sygnaly 3,3 V. Glowny pin TXD GPIO17 jest zajety przez enkoder. GPIO0 do GND sluzy do pierwszego wgrywania; odlacz te zworke przed normalnym uruchomieniem Ethernetu.

## Arduino IDE

1. Zachowaj folder WT32_L298N_AS5600 z plikami .ino, motion.h i secrets.example.h.
2. Skopiuj swoj dotychczasowy secrets.h suszarni do tego folderu albo skopiuj secrets.example.h jako secrets.h i uzupelnij dane.
3. Uzywane makra to DRYER_BOOTSTRAP_WIFI, DRYER_BOOTSTRAP_WIFI_PASSWORD, DRYER_BOOTSTRAP_OTA_PASSWORD oraz DRYER_WIFI_NETWORKS. Ostatnie to tekst JSON tablicy ssid/password. Obslugiwanych jest do 6 roznych sieci, lacznie z bootstrap. Puste lub CHANGE_ME haslo wylacza WWW i OTA.
4. Plytka ESP32 Dev Module, Flash 4 MB, PSRAM Disabled, monitor 115200. Pakiet esp32 by Espressif Systems 3.3.0; biblioteka ArduinoJson 7.4.2.
5. Partition Scheme wybierz Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS), identyfikator min_spiffs. Pierwsze wgranie przez USB-UART.

Nowy WT32 ma wlasna pamiec. Raz ustaw klapke na zero i zapisz ZERO przez USB lub przyciskiem na stronie. Restart odczytuje kat z magnesu, bez ruchu bazowania. Ruch odbywa sie najkrotsza droga. 360 stopni oznacza te sama pozycje co 0. Program nie liczy pelnych obrotow po zaniku zasilania.

Parametry domyslne SPEED 12 stopni/s (2 obr./min), ACCEL 12 stopni/s^2, tolerancja odczytu 0,1 stopnia. Zachowano profil przyspieszania/hamowania i korekty z programu testowego v2.1. Parametry dobierz do rzeczywistego napedu; tolerancja programu nie potwierdza dokladnosci mechanicznej. Czujnik AS5600 ma krok ok. 0,088 stopnia.

## Wspolpraca z obecna suszarnia

Sprawdzono plik firmware/suszarnia/shared/chute_lift.h w repo Miro199315/suszarnia, galaz dryer-io-thermal-20261030. Odbiornik laczy sie na 192.168.1.13:502, Unit ID 1. Najpierw wysyla FC16 do rejestrow 0..1 wartosci [0,0], potem co 100 ms [kierunek,PWM]. Kierunek 1 oznacza gore, 2 dol. PWM 0..1000. Odbiornik juz rozpedza rynne przez 2 sekundy. Firmware WT32 wykonuje ten sam protokol i zwraca 12-bajtowe potwierdzenie oczekiwane przez odbiornik. Zamkniecie sesji zatrzymuje nalezacy do niej naped. Brak odswiezania polecenia przez 1500 ms zatrzymuje rynne i wymaga ponownego STOP przed ruchem. Zmiana kierunku ma minimum 700 ms przerwy od wylaczenia wyjsc.

Wybrany zakres tej wersji obejmuje zadawanie klapki przez Modbus i lokalna strone. Automatyczne przelaczanie klapki wedlug etapow palnika wymaga wywolan tych rejestrow z odbiornika suszarni. Obecny panel suszarni steruje rynna swoim dotychczasowym protokolem.

## Mapa Modbus TCP

Port 502, Unit ID 1. Obslugiwane FC03, FC04, FC06 i FC16. Maksymalnie 3 klientow. Adresy ponizej sa liczone od zera; rejestr 0 bywa pokazywany jako 40001.

| Adres | Dostep | Znaczenie |
| --- | --- | --- |
| 0 | RW | Kierunek rynny 0 STOP, 1 gora, 2 dol |
| 1 | RW | PWM zadane rynny 0..1000 |
| 2 | R | Aktualne PWM rynny 0..1000 |
| 3 | R | Aktualny kierunek rynny |
| 4 | R | Rynna odblokowana po STOP, 0/1 |
| 5 | R | Wersja firmware 300 |
| 6 | RW | Kat zadany klapki 0..3600, jednostka 0,1 stopnia |
| 7 | W | Polecenie klapki 0 STOP, 1 jedz do rejestru 6, 2 ZERO, 3 zapisz SPEED i ACCEL |
| 8 | R | Aktualny kat klapki 0..3599, jednostka 0,1 stopnia |
| 9 | R | Flagi klapki, opis nizej |
| 10 | RW | SPEED 10..120, jednostka 0,1 stopnia/s |
| 11 | RW | ACCEL i hamowanie 10..1200, jednostka 0,1 stopnia/s^2 |
| 12 | R | Blad klapki 0 brak, 1 blad ruchu/czujnika, 2 utrata sterowania, 3 blad zapisu zera |
| 13 | RW | Heartbeat klapki, dowolna wartosc z tej samej sesji TCP co polecenie jazdy |
| 14 | W | 1 zatrzymuje oba napedy i blokuje rynne do nowego STOP |
| 15 | R | Surowy odczyt AS5600 0..4095 |

Flagi rejestru 9: bit 0 PWM gotowe, bit 1 czujnik OK, bit 2 zero zapisane, bit 3 ruch klapki, bit 4 cel osiagniety w tolerancji, bit 5 blad, bit 6 aktualizacja OTA. Odczyt rejestrow 7 i 14 zwraca 0.

Rynna i klapka maja osobne sesje wlascicieli. Sesja rynny moze sie zamknac po puszczeniu przycisku bez przerywania klapki sterowanej przez innego klienta.

Przyklad dla klapki 35 stopni: FC16 do adresu 6, dwie wartosci [350,1]. W tej samej sesji TCP zapisuj rejestr 13 co 500 ms az do zakonczenia przejazdu. Zanik heartbeat przez 1500 ms, utrata polaczenia albo odczytu czujnika zatrzymuje klapke. Sam odczyt statusu nie odnawia heartbeat. Nowe polecenie ruchu jest wymagane do ponownego uruchomienia. Parametry zmieniaj na postoju, np. rejestr 10=120 oraz 11=240 dla SPEED 12 i ACCEL 24. Zapisz ustawienia poleceniem 3 do rejestru 7.

## Strona lokalna i OTA przez Wi-Fi

Otworz http://192.168.1.13/ przez Ethernet lub http://ADRES_WIFI/ przez Wi-Fi. Adres Wi-Fi jest wypisywany na USB. Login admin, haslo DRYER_BOOTSTRAP_OTA_PASSWORD. Strona pozwala zadac kat, zapisac zero i parametry oraz zatrzymac oba napedy. Otwarta strona odswieza heartbeat ruchu, ktory sama uruchomila. Zamkniecie strony zatrzymuje taki ruch po maksymalnie 1500 ms.

Przegladarkowe OTA jest pod /update. W Arduino IDE wybierz Szkic > Eksportuj skompilowany program. Wgraj plik aplikacji WT32_L298N_AS5600.ino.bin. Zapis do drugiej partycji zaczyna sie po uwierzytelnieniu i zatrzymaniu obu napedow. Przerwany zapis pozostawia dotychczasowa aplikacje, jesli nie zakonczono poprawnie wyboru nowej partycji. Uruchomienie nowego firmware wymaga fizycznego sprawdzenia; nie dodano automatycznej oceny dzialania nowej aplikacji ani gwarancji powrotu po jej zawieszeniu.

Dostepne jest takze ArduinoOTA z Arduino IDE, host suszarnia-klapka, to samo haslo. Pierwsze wgranie i zmiane ukladu partycji wykonaj przez USB-UART. Aktualizacja zatrzymuje napedy; restart pozostawia je w stanie zatrzymanym. Protokol Modbus i lokalna strona wymagaja nowych polecen po aktualizacji. Parametry i zero zapisane w NVS pozostaja przy aktualizacji bez kasowania pamieci.

## USB

115200, Nowa linia. Dostepne STATUS, ZERO, kat jako liczba, SPEED 12, ACCEL 12, SAVE, STOP. SAVE zapisuje parametry do NVS. USB sluzy do testow; polecenie ruchu z USB ma wlasny limit czasu i nie wymaga sieciowego heartbeat.

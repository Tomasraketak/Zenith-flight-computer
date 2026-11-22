# Zenith v2.4 — pinout a zapojení (Hardware & Pinout Reference)

Zdroj pravdy: [`hardware/zenith_netlist_v2_4.txt`](../hardware/zenith_netlist_v2_4.txt)
(+ `Zenith_v2_4.kicad_sch`, `Zenith_v2_4_BOM_JLCPCB.csv`). Všechno níže je z něj
přepsané a ověřené proti firmwaru (`firmware/Zenith_Master`, `firmware/Zenith_Position`).

Deska má **dva mikrokontroléry**:

| MCU | modul | role | firmware |
|---|---|---|---|
| **Master** (U15) | Raspberry Pi Pico 2 (RP2350) | stavový automat mise, pyro, pogo UART s rampou, LoRa, flash log, PCA9685/INA226/ADS1115, USB diagnostika | `firmware/Zenith_Master/Zenith_Master.ino` (board `rpipico2`) |
| **Position** (U16) | Seeed XIAO RP2350 | IMU ×2, barometr, magnetometr, GPS, navigační filtr | `firmware/Zenith_Position/Zenith_Position.ino` (board `seeed_xiao_rp2350`) |

---

## 0. Rychlá mapa pro AI / strojové čtení

```yaml
# Zenith v2.4 – netlist-verified (zenith_netlist_v2_4.txt), firmware 1.00
board: Zenith_v2_4
mcus:
  master:   # U15 Raspberry Pi Pico 2, sketch firmware/Zenith_Master
    uart0_position: {tx: GP0, rx: GP1, baud: 921600, peer: "XIAO GP1/GP0 via R75/R76 1k"}
    spi0_lora:      {sck: GP2, mosi: GP3, miso: GP4, cs: GP5, dio0: GP6, rst: "PCA9685 LED8 (LOW=reset)",
                     ic: "M1 Ai-Thinker Ra-02 SX1278 433 MHz"}
    rbf_sense:      {pin: GP7, logic: "LOW = SW1 NC closed = pyro rail CAN be armed (RBF out); HIGH = safe",
                     pullup: "R32 100k to 3V3_COM, D6 BAT54 to SW1"}
    uart1_umbilical:{tx: GP8, rx: GP9, baud: 115200, conn: "J1 pogo pin3 TX / pin4 RX (R1/R2 100R, D2 USBLC6, R3 10k PU on RX)"}
    spi1_flash:     {sck: GP10, mosi: GP11, miso: GP12, cs: GP13, ic: "U17 W25Q128JVSIQ 16 MB"}
    i2c1_bus:       {sda: GP14, scl: GP15, clk_khz: 400, pullup: "R82/R83 4.7k to 3V3_COM",
                     devs: {ina226_vbat: 0x40, pca9685: 0x41, ads1115: 0x48}}
    pyro_fire:      {ch1: GP16, ch2: GP17, ch3: GP18, driver: "AO3400A low side Q5/Q6/Q7 via 100R, 10k PD", logic: "HIGH = fire"}
    pyro_arm:       {pin: GP19, driver: "Q4 AO3400A -> Q3 AO4407A high side -> PYRO_RAIL = VBAT", needs: "RBF out (SW1 NC closed)"}
    breakwire:      {bw1: GP20, bw2: GP21, logic: "LOW = wire intact, HIGH = broken", pullup: "R87/R88 10k to 3V3_COM", conn: "J8/J9"}
    cam_tx:         {pin: GP22, use: "PIO UART 115200 -> R47 1k -> J5.3 RunCam Split 4 RX (RunCam device protocol)"}
    xiao_sync:      {pin: GP26, dir: out, peer: "XIAO GP17 (D13) via R77 1k", use: "HIGH = in flight"}
    rgb:            {pin: GP27, ic: "LED1 WS2812B-2020 via R85 100R, 5V_SYS"}
    umb_present:    {pin: GP28, adc: ADC2, scale: "VIN_5V / 2 (R4/R5 100k/100k)", present_if: "> 1.8 V"}
    power:          {vsys: "PICO_VSYS <- D4 1N5819 <- 5V_SYS", vbus: NC, note: "USB powers ONLY the Pico; peripherals need battery or pogo"}
  position: # U16 Seeed XIAO RP2350, sketch firmware/Zenith_Position (uses TriSenseZenith.h)
    spi0_imu:       {sck: "GP2 (D8)", mosi: "GP3 (D10)", miso: "GP4 (D9)", cs: "GP5 (D3, R65 10k PU)", int1: "GP27 (D1)",
                     ic: "U11 LSM6DSV32X +-32 g / 4000 dps"}
    spi1_highg:     {sck: "GP10 (D17)", mosi: "GP11 (D15)", miso: "GP12 (D16)", cs: "GP9 (D18, R66 10k PU)", int1: "GP28 (D2)",
                     ic: "U10 LSM6DSV320X +-16 g / +-320 g"}
    i2c1_nav:       {sda: "GP6 (D4)", scl: "GP7 (D5)", clk_khz: 400, pullup: "R63/R64 4.7k to 3V3_NAV",
                     devs: {mmc5983ma: 0x30, bmp580: 0x46}, baro_int: "GP26 (D0)"}
    uart1_gps:      {tx: "GP20 (D12) -> GPS RXD", rx: "GP21 (D11) <- GPS TXD", pps: "GP16 (D14)",
                     ic: "U14 u-blox MAX-M10S, UBX NAV-PVT 5 Hz, dyn model AIR4", antenna: "J6 U.FL, 3V3 bias-tee (R74 10R, L3 27nH)"}
    uart0_master:   {tx: "GP0 (D6)", rx: "GP1 (D7)", baud: 921600}
    xiao_sync_in:   {pin: "GP17 (D13)"}
    rgb:            "onboard WS2812 GP22, power GP23"
    power:          "XIAO_5V <- D5 1N5819 <- 5V_SYS; XIAO 3V3 pin NC"
pca9685_0x41:     # U18, /OE = GND, outputs default LOW after power-up
  LED0..LED5: "SERVO1..6 -> R57..R62 100R -> J10/J11 signals (unused: no active control)"
  LED6: "CAM_EN -> U9 TPS22918 -> 5V_CAM (J5.1)  HIGH = camera on"
  LED7: "SERVO_PWR_EN -> Q10 -> Q9 AO4407A -> VSERVO (= VBAT!)  HIGH = servo power"
  LED8: "LORA_RST -> M1.4  LOW = reset (firmware sets full ON)"
  LED9: "BUZZER_EN -> Q12 -> BZ1 MLT-9650 (magnetic, firmware drives ~1.5 kHz PWM)"
  LED10: "FORCE_BAT -> Q11 pulls TPS2116 PR1 low -> 5V_SYS from battery buck"
  LED11..LED15: NC
ads1115_0x48:     # U19, PGA +-4.096 V; every input is a 100k/47k divider (x0.3197)
  AIN0: "CONT1 = J7- node (pyro CH1)"
  AIN1: "CONT2 = J12- node (pyro CH2)"
  AIN2: "CONT3 = J13- node (pyro CH3)"
  AIN3: "PYRO_SENSE = PYRO_RAIL"
ina226_0x40:      {shunt: "R9 5 mOhm 2512 (VBAT_P -> VBAT)", cal: 1024, lsb: "1 mA, 1.25 mV bus"}
connectors:
  J1_pogo:   {1: GND, 2: "5V in (POGO_5V -> F1 2A PTC -> VIN_5V)", 3: "TX (Master GP8)", 4: "RX (Master GP9)"}
  J2_xt60:   "2S LiPo"
  J14 / SW2: "external switch / MHR-10 rotary: SW_PWR to GND turns the battery on (any position except 0)"
  J5_runcam: {1: 5V_CAM, 2: GND, 3: "RX_CAM <- R47 <- GP22"}
  J6:        "U.FL GPS antenna (active antenna supplied 3V3)"
  J7_J12_J13:{"+": "PYRO_RAIL via 2x 4.7R 2W parallel (2.35 R)", "-": "low-side FET drain", ch: "J7=CH1 drogue, J12=CH2 main, J13=CH3 backup"}
  J8_J9:     "breakwire: pin1 signal, pin2 GND"
  J10_J11:   "servo headers S/V/G x6 + 2 spare (V = VSERVO switched battery)"
  SW1:       "RBF switch DM1-135P: COM=GND, NC -> arming path"
pyro_use:  {CH1: "drogue at apogee", CH2: "main at main_alt_m", CH3: "BACKUP charges of BOTH chutes (fired on detected failure)"}
```

---

## 1. Master MCU — Raspberry Pi Pico 2 (U15)

| Pin | GPIO | Síť | Funkce | Konstanta ve firmwaru |
|---|---|---|---|---|
| 1 | GP0 | `LINK_TX` → R75 1k → XIAO GP1 | UART0 TX → Position MCU (921600) | `PIN_LINK_TX` |
| 2 | GP1 | `LINK_RX` ← R76 1k ← XIAO GP0 | UART0 RX ← Position MCU | `PIN_LINK_RX` |
| 4 | GP2 | `LORA_SCK` | SPI0 SCK → Ra-02 | `PIN_LORA_SCK` |
| 5 | GP3 | `LORA_MOSI` | SPI0 TX | `PIN_LORA_MOSI` |
| 6 | GP4 | `LORA_MISO` | SPI0 RX | `PIN_LORA_MISO` |
| 7 | GP5 | `LORA_CS` (R78 10k PU) | Ra-02 NSS | `PIN_LORA_CS` |
| 9 | GP6 | `LORA_DIO0` | Ra-02 DIO0 | `PIN_LORA_DIO0` |
| 10 | GP7 | `RBF_NC_SENSE` (R32 100k PU, D6) | **LOW = RBF venku, lištu lze zapnout** | `PIN_RBF` |
| 11 | GP8 | `UMB_TX` → R1 100R → **J1.3** | UART1 TX → rampa (115200) | `PIN_UMB_TX` |
| 12 | GP9 | `UMB_RX` ← R2 100R ← **J1.4** (R3 10k PU) | UART1 RX ← rampa | `PIN_UMB_RX` |
| 14 | GP10 | `FLASH_SCK` | SPI1 SCK → W25Q128 | `PIN_FL_SCK` |
| 15 | GP11 | `FLASH_MOSI` | SPI1 TX | `PIN_FL_MOSI` |
| 16 | GP12 | `FLASH_MISO` | SPI1 RX | `PIN_FL_MISO` |
| 17 | GP13 | `FLASH_CS` (R79 PU) | flash /CS | `PIN_FL_CS` |
| 19 | GP14 | `I2C_SDA` (R82 4.7k) | I2C1 SDA – INA226, PCA9685, ADS1115 | `PIN_I2C_SDA` |
| 20 | GP15 | `I2C_SCL` (R83 4.7k) | I2C1 SCL | `PIN_I2C_SCL` |
| 21 | GP16 | `FIRE1` → R35 → Q5 | **PYRO1 (drogue)**, HIGH = zážeh | `PIN_FIRE[0]` |
| 22 | GP17 | `FIRE2` → R39 → Q6 | **PYRO2 (main)** | `PIN_FIRE[1]` |
| 24 | GP18 | `FIRE3` → R43 → Q7 | **PYRO3 (záloha obou padáků)** | `PIN_FIRE[2]` |
| 25 | GP19 | `PYRO_SW_ARM` → R30 → Q4 → Q3 | zapnutí pyro lišty (VBAT → `PYRO_RAIL`) | `PIN_ARM` |
| 26 | GP20 | `LAUNCH_BW1` (R87 10k PU, J8) | breakwire 1, LOW = celý | `PIN_BW1` |
| 27 | GP21 | `LAUNCH_BW2` (R88 10k PU, J9) | breakwire 2, LOW = celý | `PIN_BW2` |
| 29 | GP22 | `CAM_TX` → R47 1k → J5.3 | PIO UART 115200 → RunCam (start/stop nahrávání) | `PIN_CAM_TX` |
| 31 | GP26 | `XIAO_SYNC` → R77 1k → XIAO GP17 | HIGH = let (tvrdá linka pro Position MCU) | `PIN_SYNC` |
| 32 | GP27 | `RGB_DIN` → R85 → LED1 | WS2812B stavová LED | `PIN_RGB` |
| 34 | GP28 | `UMB_PRESENT` = VIN_5V/2 | ADC2: přítomnost 5 V z poga | `PIN_UMB_SENSE` |
| 39 | VSYS | `PICO_VSYS` ← D4 ← 5V_SYS | napájení Pica | — |
| 40 | VBUS | NC | USB napájí **jen Pico** (periférie potřebují baterii/pogo) | — |

### I²C1 (Wire1, 400 kHz)

| Adresa | IC | Použití |
|---|---|---|
| `0x40` | U2 INA226 | proud a napětí baterie (bočník R9 5 mΩ, CAL 1024 → 1 mA/LSB) |
| `0x41` | U18 PCA9685 | spínané výstupy (tabulka níže), `/OE` = GND |
| `0x48` | U19 ADS1115 | AIN0–2 kontinuita pyro CH1–3, AIN3 napětí pyro lišty (dělič 100k/47k = ×0.3197) |

### PCA9685 (U18, 0x41)

| Kanál | Síť | Funkce |
|---|---|---|
| LED0–LED5 | `SERVO1..6` → R57–R62 → J10/J11 | serva — **nepoužito** (žádné aktivní řízení) |
| LED6 | `CAM_EN` → U9 TPS22918 | napájení kamery 5V_CAM |
| LED7 | `SERVO_PWR_EN` → Q10 → Q9 | VSERVO (= VBAT) pro serva |
| LED8 | `LORA_RST` → M1.4 | reset LoRa (LOW = reset; firmware drží HIGH) |
| LED9 | `BUZZER_EN` → Q12 → BZ1 | magnetický bzučák MLT-9650 (PWM ~1.5 kHz) |
| LED10 | `FORCE_BAT` → Q11 → PR1 TPS2116 | přepnutí 5V_SYS na baterii (i když je pogo) |
| LED11–15 | NC | — |

Po zapnutí jsou všechny výstupy PCA9685 LOW → kamera vyp, serva bez napájení, LoRa v resetu, FORCE_BAT vyp.

## 2. Position MCU — Seeed XIAO RP2350 (U16)

| Pad | GPIO | Síť | Funkce |
|---|---|---|---|
| D0 | GP26 | `BARO_INT` | BMP580 INT |
| D1 | GP27 | `IMU_INT1` | LSM6DSV32X INT1 |
| D2 | GP28 | `HG_INT1` | LSM6DSV320X INT1 |
| D3 | GP5 | `IMU_CS` (R65 10k PU) | SPI0 CS LSM6DSV32X |
| D4 | GP6 | `NAV_SDA` (R64 4.7k) | I2C1 SDA – BMP580 `0x46`, MMC5983MA `0x30` |
| D5 | GP7 | `NAV_SCL` (R63 4.7k) | I2C1 SCL |
| D6 | GP0 | `LINK_TX_X` → R76 → Pico GP1 | UART0 TX → Master |
| D7 | GP1 | `LINK_RX_X` ← R75 ← Pico GP0 | UART0 RX ← Master |
| D8 | GP2 | `SPI0_SCK` | LSM6DSV32X SCK |
| D9 | GP4 | `SPI0_MISO` | LSM6DSV32X SDO |
| D10 | GP3 | `SPI0_MOSI` | LSM6DSV32X SDI |
| D11 | GP21 | `GPS_TXD` | UART1 RX ← MAX-M10S TXD |
| D12 | GP20 | `GPS_RXD` | UART1 TX → MAX-M10S RXD |
| D13 | GP17 | `XIAO_SYNC_X` ← R77 ← Pico GP26 | vstup „v letu“ |
| D14 | GP16 | `GPS_PPS` | timepulse |
| D15 | GP11 | `SPI1_MOSI` | LSM6DSV320X SDI |
| D16 | GP12 | `SPI1_MISO` | LSM6DSV320X SDO |
| D17 | GP10 | `SPI1_SCK` | LSM6DSV320X SCK |
| D18 | GP9 | `HG_CS` (R66 10k PU) | SPI1 CS LSM6DSV320X |
| 5V | — | `XIAO_5V` ← D5 ← 5V_SYS | napájení |
| 3V3 | — | NC | — |

Senzory startuje `beginZenith()` z knihovny
[VoltinoTrisenseV2](https://github.com/Tomasraketak/VoltinoTrisenseV2) (`TriSenseZenith.h`),
která obě IMU slučuje do jednoho proudu (±320 g).

## 3. Konektory

| Ref | Konektor | Piny |
|---|---|---|
| **J1** | pogo 4pin (EDAC 685) – **umbilical k rampě** | 1 GND · 2 5 V vstup (F1 PTC 2 A → VIN_5V) · 3 TX (← Pico GP8) · 4 RX (→ Pico GP9) |
| J2 | XT60 | baterie 2S (ochrana proti přepólování Q1, vypínač Q8) |
| J14 / SW2 | JST-PH 2 / MHR-10 | vypínač baterie: `SW_PWR` na GND = zapnuto (SW2 jakákoli poloha mimo 0) |
| J5 | JST-PH 3 | RunCam Split 4: 1 5V_CAM · 2 GND · 3 RX kamery |
| J6 | U.FL | GPS anténa (aktivní, napájena 3V3 přes R74/L3) |
| **J7** | svorkovnice 5.08 | **PYRO CH1 – drogue**: + = PYRO_RAIL přes 2× 4.7 Ω ‖, − = Q5 |
| **J12** | svorkovnice 5.08 | **PYRO CH2 – main** (Q6) |
| **J13** | svorkovnice 5.08 | **PYRO CH3 – záloha drogue + main** (Q7) |
| J8, J9 | svorkovnice 2.54 | breakwire 1, 2: 1 signál · 2 GND |
| J10, J11 | 3×4 kolíky | serva S/V/G (V = VSERVO = spínaná baterie) – nepoužito |
| SW1 | DM1-135P | RBF spínač |
| BT1 | MS621FE (DNP) | záloha GPS (hot start) |

## 4. Napájení

```
2S LiPo J2 ─ Q1 (přepólování) ─ Q8 (SW_PWR) ─ R9 5 mΩ (INA226) ─ VBAT ─┬─ U4 TPS563201 → 5V_BAT ─┐
                                                                       ├─ Q3 → PYRO_RAIL (pyro)   │
                                                                       └─ Q9 → VSERVO (serva)      │
pogo J1.2 ─ F1 2 A PTC ─ VIN_5V ───────────────────────────────────────────────────────────────── U1 TPS2116 ─ 5V_SYS
                                         (VIN_5V má přednost; FORCE_BAT → PR1 low → baterie)       │
5V_SYS ─┬─ U6 ME6211 → 3V3_COM  (LoRa, flash, INA/ADS/PCA, pull-upy)
        ├─ U8 ME6211 → 3V3_NAV  (IMU, baro, mag, GPS)
        ├─ D4 → Pico VSYS,  D5 → XIAO 5V
        ├─ U9 → 5V_CAM (CAM_EN),  BZ1, LED1
```

Důsledky pro firmware a obsluhu:

* **Na rampě raketa běží z rampy** (5 V po pogu); baterie se nevybíjí.
* V odpočtu v **T-`cd_force_bat_s`** (výchozí 10 s) Master zapne `FORCE_BAT` a do 1 s
  ověří na INA226, že z baterie teče proud (> 40 mA). Když ne (vypnutý vypínač baterie,
  vadná baterie), **odpočet se přeruší** (`ZE_BATT_SWITCH`) — raketa by po odpojení poga
  ztratila napájení.
* USB kabel napájí jen Pico (VBUS na desce nezapojen); pro diagnostiku periférií je
  potřeba zapnout baterii nebo připojit rampu.

## 5. Pyro obvody

* Zapnutí lišty: `PYRO_SW_ARM` (GP19) HIGH → Q4 sepne gate Q3 **jen když je RBF venku**
  (SW1 COM–NC sepnuto). S RBF uvnitř nejde lištu zapnout ani softwarovou chybou.
* Odpálení: `FIREx` HIGH → low-side AO3400A. Proud je omezen 2× 4.7 Ω paralelně (2.35 Ω,
  ~3.5 A při 8.4 V). Rezistory jsou 2 W — **pulz drž krátký** (`pyro_pulse_ms`, výchozí
  300 ms); rozbuška se zapálí za < 10 ms.
* Kontinuita: přes R28 10 kΩ z VBAT teče i s vypnutou lištou slabý proud rozbuškou do
  děliče 100k/47k → ADS1115. S rozbuškou ~0.3×VBAT (≈ 2.4 V na 8 V), bez ní 0 V. Práh
  `cont_min_mv` (výchozí 300 mV na pinu ADC).

## 6. Pogo umbilical s rampou

Stejný konektor a piny jako rampa Daidalosu (Rampa PCB1, J3), jen místo I²C jede **UART 115200**:

| Zenith J1 | směr | Rampa J3 | Rampa |
|---|---|---|---|
| 1 GND | — | 1 GND | GND |
| 2 5V | ← | 2 UMB_5V | U2 buck, spínaný TCA9534 P1 `ROCKET_EN` |
| 3 TX (Pico GP8) | → | 3 | R22 100R → **GP1 = UART0 RX** |
| 4 RX (Pico GP9) | ← | 4 | R21 1k ← **GP0 = UART0 TX** |

Protokol: `firmware/common/zenith_link.h` (`ZL_PKT_RMP_CMD` / `ZL_PKT_RMP_STATUS`),
firmware rampy je v repozitáři [RampaCode](https://github.com/Tomasraketak/RampaCode).

## 7. Pozemní stanice

Stejná deska a pinout jako u Daidalosu (Pico 2 W): LoRa Ra-02 na SPI1 (MISO GP12, CS GP11,
SCK GP14, MOSI GP15, RST GP20, DIO0 GP21), OLED SSD1306 `0x3C` na I2C0 (GP4/GP5), tlačítka
BL GP3 / BC GP6 / BR GP10 (aktivní HIGH). Firmware `firmware/Ground_Station` (board `rpipico2w`).

## 8. Rádio

433 MHz, SF7, BW 250 kHz, CR 4/5, sync word `0x2B`, +20 dBm (`zenith_link.h`). Telemetrie
52 B = 51 ms vysílání; výchozí 5 Hz (26 % času), na zemi 2 Hz. **Zkontroluj si limity
pásma 433 MHz (ETSI EN 300 220: 10 % duty / 10 mW ERP) a pravidla soutěže.**

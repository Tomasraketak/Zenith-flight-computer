# USB protokoly (textové řádky, 115200 Bd, `\r\n`)

Oba GUI jsou jen klienti těchto protokolů — vše jde dělat i ze Serial Monitoru.

## 1. Master MCU (letový počítač na kabelu) — `gui/zenith_diag.py`

Master posílá jen když je port otevřený a nikdy kvůli USB neblokuje (řádek, který se
nevejde do bufferu, zahodí).

### Výstup

| Řádek | Frekvence | Pole |
|---|---|---|
| `$ID,Zenith Master,<fw>,<počet param>` | na `HELLO` | |
| `$S,...` | 10 Hz | state, err, tlm_flags, health, t_ms, vbat_mv, curr_ma, pyro, ads0..ads3 [mV na pinu ADC], rampa_state, rampa_flags, rampa_faults, rampa_vbat_mv, t_rem_ms, bw1, bw2, umb, rbf_armable, umb_mv, lora_ok, flash_ok, main_alt_m, log_head, log_dropped, log_erasing, reset_reason |
| `$N,...` | 20 Hz | alt, vz, px, py, vx, vy, acc_axial_g, acc_mag_g, mach, baro_alt, sig_alt, tilt_deg, qw, qx, qy, qz, max_alt, nav_flags, nav_mode |
| `$P,...` | 5 Hz | lat, lon, gps_alt, sats, fix, hacc_m, pressure_pa, baro_temp, roll, pitch, yaw, gbias×3, peak_g, imu_hz, sensors, nose_axis, imu_lost, fw, reset, pad_lat, pad_lon |
| `$R,...` | 25 Hz (po `RAW 1`) | ax, ay, az [g], gx, gy, gz [°/s], mx, my, mz [µT], high-g podíl |
| `$F,...` | 1 Hz | why_drogue, why_main, why_p3, fired_mask, max_alt, max_vz, max_mach, max_g, pyro_events |
| `$PRM,i,name,value,min,max,default,popis` | na `PARAM LIST`/`SET` | |
| `$EV,t_ms,text` | při události | (stejný text jde do flash logu) |
| `$LOGBEGIN,n` / `$LOG,<128 hex>` / `$LOGEND,n` | na `LOG DUMP` | záznamy 64 B (`LogData` 0xA7 / `LogEvent` 0xA8, CRC16) |
| `$LOGINFO,head,capacity,dropped,erasing,jedec` | na `LOG INFO` | |
| `#OK …` / `#ERR …` / `#RC <cmd> <rc>` | odpověď | rc = `zl_ul_rc_t` |

Bitové masky (`zenith_link.h`): `ZL_TF_*` (tlm_flags), `ZL_HL_*` (health), `ZL_NAV_*`,
`ZL_SENS_*`, `ZL_RMP_*`, `ZL_RMPF_*`. `pyro`: bit0–2 kontinuita CH1–3, bit3–5 vypáleno,
bit6 lišta zapnuta.

### Příkazy

| Příkaz | Kdy | Popis |
|---|---|---|
| `HELLO` | vždy | identifikace |
| `PARAM LIST` / `PARAM SET <name> <v>` / `PARAM SAVE` / `PARAM DEFAULTS` | ne v odpočtu/letu | tabulka `zenith_params.h`, SAVE zapíše do flash (A/B sektory) |
| `PREPARE`, `COUNTDOWN <s>`, `ABORT`, `SAFE`, `PADREF`, `LOCATE 0/1`, `CAM 0/1`, `REBOOT` | jako z LoRa | stejná logika jako příkazy ze země |
| `GYROCAL`, `GPSRECFG`, `POSREBOOT` | na zemi | Position MCU |
| `RAW 0/1` | vždy | proud `$R` |
| `DIAG 1` / `DIAG 0` | z IDLE/ABORT | testovací režim |
| `ARM 0/1`, `FIRE <1..3> CONFIRM`, `FORCEBAT 0/1`, `SERVOPWR 0/1` | jen v DIAG | pyro test vyžaduje RBF venku |
| `BUZZ <ms>`, `RAMPA PING`, `FLIGHT` | vždy | |
| `LOG INFO` / `LOG DUMP` / `LOG ERASE CONFIRM` | dump/erase jen na zemi | |

## 2. Pozemní stanice — `gui/ground_station_gui.py`

### Výstup

| Řádek | Pole |
|---|---|
| `$T,...` (telemetrie rakety) | rssi, snr, state, flags, t_ms, lat, lon, alt_m, t_rem_ms (odpočet; v letu čas od startu), vz, vh, acc_g, max_alt, vbat_V, next_rx_ms, main_alt, tilt, mach, sats, pyro, health, rampa_state, rampa_flags, last_ul_seq, last_ul_rc, err, win_final |
| `$B,...` (beacon rampy bez rakety) | rssi, snr, state, flags, t_ms, vbat_V, umb_ma, cont1..3_mV, next_rx_ms, faults, last_ul_seq, last_ul_rc, fw |
| `$U,seq,CMD,dest,STAV,rc,pokus` | STAV = QUEUED / SENT / SENT2 / ACK / FAILED / CANCELLED |
| `$G,rx_ok,rx_bad,tlm_age_ms,win_locked,win_in_ms,pending,win_dest,beacon_age_ms` | 1 Hz |

### Příkazy

`CMD <NAME> [arg] [RAMPA]`, kde NAME = `NOP PREPARE COUNTDOWN ABORT SAFE PADREF MAINALT LOCATE CAMERA REBOOT ROCKETPWR`
(`COUNTDOWN` v sekundách, `MAINALT` v metrech, `ROCKETPWR 0/1` jde vždy rampě),
`CANCEL`, `TEST 0|1` (simulovaná telemetrie), `HELLO`.

Stanice drží jeden příkaz; odešle ho v nejbližším RX okně odesílatele (raketa, nebo
rampa když raketa nevysílá), ABORT a COUNTDOWN dvakrát v témž okně, a čeká na potvrzení
v `last_ul_seq/last_ul_rc` další telemetrie. Bez potvrzení to zkusí v dalších oknech
(max. 3). Raketa duplicitní `seq` ignoruje, opakování je tedy bezpečné.

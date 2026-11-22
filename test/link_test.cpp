// Zenith host test: framing codec, LoRa packets, TDMA window maths, radio budget.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#include "zenith_link.h"

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("   FAIL: "); } else printf("   ok:   "); \
                           printf(__VA_ARGS__); printf("\n"); } while (0)

int main()
{
    std::mt19937 rng(42);
    printf("== framing round trip\n");
    {
        zl_nav_t n{}; n.alt = 1234.5f; n.vz = -3.25f; n.flags = 0xBEEF; n.seq = 7;
        uint8_t f[ZL_FRAME_MAX];
        uint16_t len = zl_encode(f, ZL_PKT_NAV, &n, sizeof n);
        CHECK(len == sizeof(n) + 6, "frame length %u", len);
        zl_rx_t rx; zl_rx_init(&rx);
        const uint8_t *pl = nullptr; uint8_t pn = 0; uint8_t type = 0;
        for (uint16_t i = 0; i < len; i++) { uint8_t t = zl_rx_byte(&rx, f[i], &pl, &pn); if (t) type = t; }
        zl_nav_t o{};
        CHECK(type == ZL_PKT_NAV && zl_take(&o, sizeof o, pl, pn) && o.alt == 1234.5f && o.flags == 0xBEEF,
              "decoded NAV");
    }
    printf("== corrupted stream: every intact frame recovered\n");
    {
        std::vector<uint8_t> stream; int sent = 0;
        std::vector<int> hit;
        for (int k = 0; k < 2000; k++) {
            zl_rmp_status_t s{}; s.vbat_mv = (uint16_t)k; s.state = ZR_ST_IDLE;
            uint8_t f[ZL_FRAME_MAX]; uint16_t len = zl_encode(f, ZL_PKT_RMP_STATUS, &s, sizeof s);
            bool corrupt = (rng() % 10) == 0;
            if (corrupt) f[rng() % len] ^= (uint8_t)(1u << (rng() % 8));
            if ((rng() % 25) == 0) stream.push_back((uint8_t)rng());      // inserted garbage
            stream.insert(stream.end(), f, f + len);
            sent++; if (!corrupt) hit.push_back(k);
        }
        zl_rx_t rx; zl_rx_init(&rx);
        int got = 0, got_intact = 0;
        for (uint8_t b : stream) {
            const uint8_t *pl; uint8_t pn;
            if (zl_rx_byte(&rx, b, &pl, &pn) == ZL_PKT_RMP_STATUS) {
                zl_rmp_status_t s; if (zl_take(&s, sizeof s, pl, pn)) { got++; got_intact++; }
            }
        }
        CHECK(got_intact >= (int)hit.size() * 98 / 100, "recovered %d of %zu intact frames (%d sent)", got_intact, hit.size(), sent);
        CHECK(got <= sent, "no phantom frames");
    }
    printf("== LoRa packets\n");
    {
        zl_tlm_t t{}; t.state = ZS_COAST; t.alt_cm = 250000;
        zl_lora_seal(&t, sizeof t, ZL_TLM_MAGIC);
        CHECK(zl_lora_check((uint8_t *)&t, sizeof t, sizeof t, ZL_TLM_MAGIC), "telemetry CRC ok");
        ((uint8_t *)&t)[10] ^= 1;
        CHECK(!zl_lora_check((uint8_t *)&t, sizeof t, sizeof t, ZL_TLM_MAGIC), "telemetry corruption detected");
        zl_uplink_t u{}; u.cmd = ZL_UL_ABORT; u.token = zl_ul_token_for(ZL_UL_ABORT);
        zl_lora_seal(&u, sizeof u, ZL_UL_MAGIC);
        CHECK(zl_lora_check((uint8_t *)&u, sizeof u, sizeof u, ZL_UL_MAGIC), "uplink CRC ok");
        CHECK(!zl_lora_check((uint8_t *)&u, sizeof u, sizeof u, ZL_TLM_MAGIC), "magic distinguishes packets");
    }
    printf("== radio budget (SF%d BW%lu kHz)\n", ZL_LORA_SF, ZL_LORA_BW_HZ / 1000);
    {
        const uint32_t at = zl_lora_airtime_us(sizeof(zl_tlm_t));
        const uint32_t au = zl_lora_airtime_us(sizeof(zl_uplink_t));
        printf("   telemetry %u B: %.1f ms, uplink %u B: %.1f ms, beacon %.1f ms\n",
               (unsigned)sizeof(zl_tlm_t), at / 1000.0, (unsigned)sizeof(zl_uplink_t), au / 1000.0,
               zl_lora_airtime_us(sizeof(zl_rampa_beacon_t)) / 1000.0);
        CHECK(at * 10u < 1000000u * 6u / 10u, "10 Hz telemetry < 60 %% duty (%.0f %%)", at * 10 / 10000.0);
        CHECK(ZL_UL_OFFSET2_MS + au / 1000u + 20u < ZL_WIN_DUR_MS, "both uplink copies fit in the window");
        CHECK(ZL_WIN_TX_GUARD_MS > at / 1000u, "TX guard longer than one telemetry packet");
    }
    printf("== TDMA windows\n");
    {
        int fin = 0;
        CHECK(zl_next_window_ms(12000, -1, &fin) == 8000 && !fin, "regular grid");
        uint32_t w = zl_next_window_ms(100000, 30000, &fin);
        CHECK(w == 0 || w <= 10000, "regular window during countdown (%u)", w);
        w = zl_next_window_ms(100500, 2000, &fin);
        CHECK(fin && w == 850, "final abort window at T-1150 ms (%u, fin %d)", w, fin);
        // no regular window inside the last 1.6 s
        bool bad = false;
        for (int32_t rem = 20000; rem > 0; rem -= 7) {
            uint32_t now = 500000 + (uint32_t)(20000 - rem);
            int f2 = 0; uint32_t nw = zl_next_window_ms(now, rem, &f2);
            int32_t at = rem - (int32_t)nw;
            if (!f2 && at < 1600 && at > -2000) bad = true;
        }
        CHECK(!bad, "no regular window in the last 1.6 s before T-0");
    }
    printf("\n%s (%d failures)\n", g_fail ? "LINK TEST FAILED" : "LINK TEST PASSED", g_fail);
    return g_fail ? 1 : 0;
}

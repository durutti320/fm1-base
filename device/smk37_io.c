/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * SMK-37 Pro port: Copyright (C) 2026 M. Geisler (genkigenki/makkuro); in fm1-base: Cam Durnsford (SPIDER) */
/* The M-VAVE SMK-37's pad LEDs and wheels (BOARD_SMK37), from genkigenki's SMK-37 layer for Felucca
 * (genkigenki/Felucca branch smk37: src/lcd.c padled_*, src/main.c smk_wheels). GPL-3.0-only like that layer, not
 * fm1-base's MPL-2.0. Part of the unity build after lcd.c (plat_smk37.c uses it). */

/* The 16 RGB pads: WS2812-class LEDs on SPI1 group B (PC10), one SPI byte per LED bit (0xF0 = 1, 0xC0 = 0, MSB first,
 * GRB), 250 zero bytes of reset first (~300 us; the stock firmware's 36 bytes = 43 us are too short for newer WS2812
 * parts: a glitch on PC10 at the pin switch then counts as a first bit and shifts the chain by one, so a lit pad turns
 * its chain neighbour green). The frame goes out between LCD transfers (LCD CS high), blocking for ~0.8 ms;
 * padled_flush() only sends when a colour changed. */
#define PADLED_N 16u
#define PADLED_LATCH 250u
static uint8_t padled_buf[PADLED_LATCH + PADLED_N * 24u];
static uint32_t padled_rgb[PADLED_N];
static uint8_t padled_dirty = 1;
/* hardware pad (0..7 top row, 8..15 bottom row, left to right) -> position in the chain (stock table) */
static const uint8_t PADLED_SLOT[PADLED_N] = {0, 15, 14, 13, 12, 11, 10, 9, 1, 2, 3, 4, 5, 6, 7, 8};

static void padled_set(uint32_t pad, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t c = (uint32_t)g << 16 | (uint32_t)r << 8 | b;
    if (pad < PADLED_N && padled_rgb[pad] != c) {
        padled_rgb[pad] = c;
        padled_dirty = 1;
    }
}

static void padled_flush(void)
{
    uint32_t p, i;
    if (!padled_dirty)
        return;
    padled_dirty = 0;
    for (p = 0; p < PADLED_N; p++) {
        uint8_t *o = padled_buf + PADLED_LATCH + PADLED_SLOT[p] * 24u;
        for (i = 0; i < 24u; i++)
            o[i] = (padled_rgb[p] >> (23u - i)) & 1u ? 0xF0u : 0xC0u;
    }
    lcd_sync();
    fm1_padled_send(padled_buf, sizeof padled_buf);
    fm1_lcd_wait();
    fm1_padled_done();
}

/* the wheels (stock: mux C address 4 = pitch, 5 = mod), self-calibrating: the pitch wheel's rest is its centre
 * (taken at power-on once the reading has stayed still, followed slowly while it rests), each wheel's range grows to
 * what it has reached (the travel is short: raw ~445..640 pitch, ~474..629 mod on a unit). Pitch: a 14-bit bend
 * -8192..8191 with a dead band around the centre; mod: 0..127, up (lower raw) = more. A centre taken wrong (a reading
 * not yet settled, the wheel touched at power-on: on a unit the rest then bent a full fifth down and only the wheel
 * held up played in tune) mends itself: the wheel left still for RECENTRE_MS near the middle of the travel it has
 * really reached (within an eighth of it either side) is its rest; a bend held further out stays. Main loop, every pass */
static volatile int16_t smk_bend14;
static volatile uint8_t smk_modw;
static void smk_wheels(void)
{
    enum { DEAD = 12, SPAN0 = 80, MOD_LO = 485, MOD_HI = 615, STILL = 8, SETTLE_MS = 300, SETTLE_MAX_MS = 2000,
           RECENTRE_MS = 3000 };
    static int32_t pc, plo, phi, mlo, mhi, rest, sv, smin, smax;   /* smin, smax: the travel really reached */
    static uint32_t st, t0;                       /* since when the pitch reading stays within STILL of sv; the first */
    int32_t p = fm1_in.wheel[0], m = fm1_in.wheel[1], d, b, mw;
    uint32_t now = plat_ms();
    if (fm1_in.frames < 300u)                     /* (the readings settle) */
        return;
    if (!t0)
        t0 = now | 1u;
    if (p - sv > STILL || sv - p > STILL) {       /* moved: still again from now */
        sv = p;
        st = now;
    }
    if (!pc) {                                    /* the centre: once the reading has been still (or after 2 s anyway) */
        if (now - st < SETTLE_MS && now - t0 < SETTLE_MAX_MS)
            return;
        pc = now - st >= SETTLE_MS ? sv : p;
        plo = pc - SPAN0;
        phi = pc + SPAN0;
        mlo = m < MOD_LO ? m : MOD_LO;
        mhi = m > MOD_HI ? m : MOD_HI;
    }
    if (!smax || p < smin)
        smin = p;
    if (p > smax)
        smax = p;
    if (p < plo)
        plo = p;
    if (p > phi)
        phi = p;
    if (m < mlo)
        mlo = m;
    if (m > mhi)
        mhi = m;
    if (now - st >= RECENTRE_MS && (sv - pc > DEAD || pc - sv > DEAD) &&
        smax - smin > 4 * SPAN0 / 3 && 8 * (sv - (smin + smax) / 2) < smax - smin &&
        8 * ((smin + smax) / 2 - sv) < smax - smin) {
        pc = sv;                                  /* still, off the centre, mid-travel: the rest; the range anew */
        plo = pc - SPAN0;
        phi = pc + SPAN0;
    }
    d = p - pc;
    if (d > -DEAD && d < DEAD) {
        b = 0;
        if (++rest > 200) {                        /* resting: the centre follows a slow drift */
            pc += d > 0 ? 1 : d < 0 ? -1 : 0;
            rest = 0;
        }
    } else {
        rest = 0;
        b = d > 0 ? (d - DEAD) * 8191 / (phi - pc - DEAD > 1 ? phi - pc - DEAD : 1)
                  : (d + DEAD) * 8192 / (pc - plo - DEAD > 1 ? pc - plo - DEAD : 1);
        b = clamp(b, -8192, 8191);
    }
    mw = mhi - m <= DEAD ? 0 : (mhi - m - DEAD) * 127 / (mhi - mlo - DEAD > 1 ? mhi - mlo - DEAD : 1);
    smk_bend14 = (int16_t)b;
    smk_modw = (uint8_t)clamp(mw, 0, 127);
}

/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Cam Durnsford (SPIDER, the LOOM-8 SMK-37 port) */
/* fm1-base on the M-VAVE SMK-37 (BOARD_SMK37): the parts of plat.h that differ from the FM-1's (plat_fm1.c includes
 * this in their place) and the SMK's extras (plat.h, PLAT_SMK37). Over felucca-bsp's boards/smk37 (genkigenki's SMK-37
 * input HAL) and smk37_io.c (its pad LEDs and wheels). The buttons, keys and knobs need nothing of their own:
 * plat_buttons, plat_keys and plat_enc read them through the panel map as on the FM-1 (the SMK's ids, panel.c), and
 * the keys are its middle 27 (the pads never press a key: SMK_PAD_EVENT, unity.c). */

/* ---- the lights: the buttons' LEDs are on / off (no glow); the keys have none; PLAY's LED is two-colour (button
 * LED 16 green, 18 red: smk_input.h SMK_LEDC) */
#define SMK_LED_PLAY_GREEN 16u
#define SMK_LED_PLAY_RED 18u
static uint32_t smk_own_lit;                      /* plat_smk_leds: the four SMK buttons' LEDs */
static void led_pos_init(void) {}
void plat_lights(uint32_t buttons, uint32_t keys, uint32_t dim_buttons, uint32_t dim_keys, int glow)
{
    uint32_t i, play = 1u << B_PLAY;
    uint8_t l[SMK_NLED] = {0};
    (void)dim_keys;
    (void)glow;
    for (i = 0; i < NB; i++)
        if (i != B_PLAY && ((buttons >> i) & 1u) && panel.btn[i] < SMK_NLED)
            l[panel.btn[i]] = 1;
    if (buttons & play)
        l[SMK_LED_PLAY_RED] = 1;                  /* (PLAY lit: the FM-1's red PLAY LED) */
    if (buttons & LED_PLAY_GREEN)
        l[SMK_LED_PLAY_GREEN] = 1;
    (void)dim_buttons;                            /* (no dim level on the SMK: unlit is off) */
    for (i = 0; i < NSB; i++)
        if ((smk_own_lit >> i) & 1u)
            l[SMK_BTN_KNOB_BANK + i] = 1;
    for (i = 0; i < SMK_NLED; i++)
        fm1_led[i] = l[i];
    {   /* the mirror's view, as on the FM-1: PLAY lit (either LED), and bit 15 PLAY lit red */
        uint32_t lit = buttons & ((1u << NB) - 1u);
        mirror_leds((lit & ~play) | (buttons & (play | LED_PLAY_GREEN) ? play : 0u) | (buttons & play ? 1u << 15 : 0u),
                    keys);
    }
}
void plat_smk_leds(uint32_t lit) { smk_own_lit = lit & ((1u << NSB) - 1u); }

/* ---- no key sensing on the SMK: its keys have velocity of their own (plat_key_vel) */
void plat_sense(int on) { (void)on; }
uint32_t plat_analog_mask(void) { return 0; }

/* ---- the extras */
uint32_t plat_smk_buttons(void)
{
    uint32_t b = fm1_in.buttons;
    return ((b >> SMK_BTN_KNOB_BANK) & 1u) << SB_KNOB_BANK | ((b >> SMK_BTN_FADER_BANK) & 1u) << SB_FADER_BANK |
           ((b >> SMK_BTN_LEFT) & 1u) << SB_LEFT | ((b >> SMK_BTN_RIGHT) & 1u) << SB_RIGHT;
}
/* a velocity 1..127: the unit's curve can give 1024 at the very top (smk_input.h smk_curve), 128 once in 7 bits,
 * which as MIDI would be a note-off */
static uint32_t smk_vel_clamp(uint32_t v) { return v > 127u ? 127u : v; }
uint32_t plat_key_vel(int k)
{
    uint32_t v = k >= 0 && k < NKEYS ? fm1_in.key_vel[SMK_KEY_F3 + (uint32_t)k] : 0u;
    return v ? smk_vel_clamp(v) : 100u;
}
/* the pads: smk_input.h hands each hit and release here (SMK_PAD_EVENT, the 10 kHz scan), a ring for the main loop */
#define SMK_PADQ 32u
static volatile uint16_t smk_padq[SMK_PADQ];
static volatile uint8_t smk_padq_w, smk_padq_r;
static volatile uint32_t smk_pads_down;
static void smk_pad_event(uint32_t pad, uint32_t vel)
{
    uint8_t w = smk_padq_w;
    if (pad >= NPADS)
        return;
    smk_pads_down = vel ? smk_pads_down | 1u << pad : smk_pads_down & ~(1u << pad);
    if ((uint8_t)((w + 1u) % SMK_PADQ) == smk_padq_r)
        return;                                   /* (full: dropped; the held mask stays right) */
    smk_padq[w] = (uint16_t)(pad << 8 | smk_vel_clamp(vel));   /* (a hit is never 0: a release) */
    smk_padq_w = (uint8_t)((w + 1u) % SMK_PADQ);
}
int plat_pad(int *pad, int *vel)
{
    uint32_t e;
    if (smk_padq_r == smk_padq_w)
        return 0;
    e = smk_padq[smk_padq_r];
    smk_padq_r = (uint8_t)((smk_padq_r + 1u) % SMK_PADQ);
    *pad = (int)(e >> 8);
    *vel = (int)(e & 0x7Fu);
    return 1;
}
uint32_t plat_pads_held(void) { return smk_pads_down; }
/* the outer keys: smk_input.h's queue (keybed key << 8 | velocity), keys 0..4 and 32..36 */
int plat_outer(int *key, int *vel)
{
    int32_t e;
    while ((e = smk_extkey_take()) >= 0) {
        uint32_t k = (uint32_t)e >> 8;
        if (k < SMK_KEY_F3)
            *key = (int)k;
        else if (k >= SMK_KEY_F3 + NKEYS && k < FM1_NKEY)
            *key = (int)(k - (SMK_KEY_F3 + NKEYS)) + 5;
        else
            continue;
        *vel = (int)smk_vel_clamp((uint32_t)e & 0xFFu);   /* (128 at the very top: kept a hit) */
        return 1;
    }
    return 0;
}
/* K1 as a knob of its own (plat_k1_raw): the main loop leaves the volume where it is (main_fm1.c) */
static volatile uint8_t smk_k1_raw;
void plat_k1_raw(int on)
{
    (void)fm1_enc_take(SMK_ENC_MASTER);           /* (turns from before: neither the volume's nor the app's) */
    smk_k1_raw = (uint8_t)(on != 0);
}
int32_t plat_k1_enc(void) { return smk_k1_raw ? fm1_enc_take(SMK_ENC_MASTER) : 0; }
uint32_t plat_fader(int f) { return f >= 0 && f < 4 ? fm1_in.fader[f] : 0u; }
int32_t plat_bend(void) { return smk_bend14; }
uint32_t plat_modw(void) { return smk_modw; }
void plat_pad_rgb(int pad, uint32_t rgb)
{
    if (pad >= 0 && pad < NPADS)
        padled_set((uint32_t)pad, (uint8_t)(rgb >> 16), (uint8_t)(rgb >> 8), (uint8_t)rgb);
}

/* ---- the data windows: the SMK's (plat.h, felucca-bsp fm1_flash_smk37.h). The stock firmware trusts USRFLASH while
 * its end marker "usrdata" (0xEAFF8) is there: its sector is erased once, before the first erase or write into
 * PLAT_XDATA0, so a return to stock rebuilds its factory mapping instead of reading the app's data as its own */
_Static_assert(FL_DATA_LO == PLAT_DATA_LO && FL_DATA_HI == PLAT_DATA_HI && FL_XDATA0_LO == PLAT_XDATA0_LO &&
               FL_XDATA0_HI == PLAT_XDATA0_HI && PLAT_XDATA1_LO == PLAT_XDATA1_HI, "plat.h's windows are felucca-bsp's");
static void smk_stock_release(uint32_t off)
{
    static const uint8_t TAG[8] = {'u', 's', 'r', 'd', 'a', 't', 'a', 0};
    const uint8_t *t = (const uint8_t *)FL_XIP(FL_STOCKTAG_OFF);
    uint32_t i, took;
    if (!plat_in(off, 1u, PLAT_XDATA0_LO, PLAT_XDATA0_HI))
        return;
    for (i = 0; i < 8u; i++)
        if (t[i] != TAG[i])
            return;
    audio_silence();
    fl_erase4k(FL_STOCKTAG_LO, &took);
}

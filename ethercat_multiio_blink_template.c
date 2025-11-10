/*
* READ FIRST
 * -----------
 * WHAT:  Minimal, didactic EtherCAT multi-I/O template (3 sample slaves: 4xDO nibble, 4xDI nibble, 16-bit OUT + 20-byte IN).
 * WHY:   Show how to (a) detect slaves by VendorId/ProductCode, (b) map PDOs to C pointers safely (bit-aligned),
 *        and (c) run a clock-based "blink" independent of the IO cycle.
 * HOW:   Replace the "Integration Layer (TODO)" stubs with your master SDK calls (map PI, SDO R/W, states, sleep, log).
 *
 * SCOPE & SAFETY
 *  - No motion control here; only digital I/O patterns. Still, changing outputs may move actuators → verify wiring first.
 *  - SDO writes are OFF by default. Guard any mailbox/SDO tweaks with macros and do them in the correct state (usually PRE-OP).
 *  - This file is platform/master-agnostic: no proprietary headers; you must provide the glue in the Integration Layer.
 *
 * HOW TO READ THIS FILE
 *  1) "User Config Knobs": tune period, enable/disable guards, choose whether to compile the optional reference block.
 *  2) "Utils": monotonic clock and small helpers for bit-aligned nibble writes.
 *  3) "PDO/Data mapping": small structs + runtime mapping into process image (PI).
 *  4) "Integration Layer (TODO)": the only place you must adapt to your master SDK.
 *  5) "Init()": discover & map; optional SDOs (guarded).
 *  6) "Run()": time-driven toggle + input mirroring with bounds checks.
 *  7) Troubleshooting comments sit near risky spots (state checks, offsets, sizes).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

/* ================================================================
 * User Config Knobs
 * ================================================================ */
#ifndef BLINK_PERIOD_MS
#define BLINK_PERIOD_MS          500u     /* Toggle period in real time */
#endif

#ifndef ENABLE_REFERENCE_BLOCK
#define ENABLE_REFERENCE_BLOCK   1        /* 1: compile the "Reference:" comments section, 0: strip it */
#endif

/* Optional, guarded tweaks — keep OFF unless you know the device allows it in PRE-OP */
#ifndef ENABLE_WATCHDOG_TWEAKS
#define ENABLE_WATCHDOG_TWEAKS   0        /* 1: attempt SDO writes to SM/PDI watchdogs; 0: off */
#endif

#ifndef ENABLE_ONE_SHOT_SDO_TEST
#define ENABLE_ONE_SHOT_SDO_TEST 0        /* 1: demo write on a DO object once at init; 0: off */
#endif

/* Demo SIM mode (no master required). Define IL_ENABLE_SIM to compile a tiny simulator. */
#ifndef IL_ENABLE_SIM
#define IL_ENABLE_SIM            0
#endif

/* ================================================================
 * Reference: example profiles (for documentation only)
 * Toggle off with ENABLE_REFERENCE_BLOCK=0
 * ================================================================ */
#if ENABLE_REFERENCE_BLOCK
/*
 * Beckhoff EL2004 (4 DO)        VID=0x00000002, PID=0x07D43052, Outputs: 4 bits (may start at any bit offset)
 * Beckhoff EL1004 (4 DI)        VID=0x00000002, PID=0x03EC3052, Inputs : 4 bits
 * Omron NX-ECC203 (example)     VID=0x00000083, PID=0x000000AA, In: ~20 bytes, Out: 16 bits (example layout)
 * WAGO 750-XXX (example)        VID=0x00000021, PID=0x07500354, In/Out: 6 bytes (example)
 *
 * IMPORTANT:
 *  - PDO bit offsets/sizes come from the ENI (or programmatic mapping). Do not assume byte alignment.
 *  - If your nibble crosses a byte boundary, write it split across two bytes (see write_nibble_at()).
 */
#endif

/* ================================================================
 * Endianness / Unaligned Helpers (portable nibble writer)
 * ================================================================ */
static inline uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* Write a 4-bit value at an arbitrary bit offset within an output process image.
 * WHY: PDOs for small DO modules are often nibble-sized and not byte-aligned.
 * HOW: mask-merge into byte0 (and byte1 if boundary is crossed).
 */
static inline void write_nibble_at(uint8_t *byte0, uint8_t *byte1, int shift_bits, uint8_t nibble /*0..15*/) {
    int s = shift_bits & 7;                        /* 0..7 */
    uint8_t v = (uint8_t)(nibble & 0x0F);

    if (byte0 == NULL) return;

    if (s <= 4) {
        /* May or may not cross the byte boundary */
        if (s + 4 <= 8) {
            /* fits into byte0 */
            uint8_t mask = (uint8_t)(0x0Fu << s);
            *byte0 = (uint8_t)((*byte0 & (uint8_t)~mask) | ((uint8_t)(v << s) & mask));
        } else {
            /* crosses into byte1 */
            if (byte1 == NULL) return;
            uint8_t low_mask  = (uint8_t)(0xFFu << s);          /* bits s..7 */
            uint8_t high_mask = (uint8_t)(0x0Fu >> (8 - s));    /* upper bits in next byte */
            uint8_t low_val   = (uint8_t)(v << s);
            uint8_t high_val  = (uint8_t)(v >> (8 - s));
            *byte0 = (uint8_t)((*byte0 & (uint8_t)~low_mask)  | (low_val  & low_mask));
            *byte1 = (uint8_t)((*byte1 & (uint8_t)~high_mask) | (high_val & high_mask));
        }
    } else {
        /* s==5..7: always crosses */
        if (byte1 == NULL) return;
        uint8_t low_mask  = (uint8_t)(0xFFu << s);
        uint8_t high_mask = (uint8_t)(0x0Fu >> (8 - s));
        uint8_t low_val   = (uint8_t)(v << s);
        uint8_t high_val  = (uint8_t)(v >> (8 - s));
        *byte0 = (uint8_t)((*byte0 & (uint8_t)~low_mask)  | (low_val  & low_mask));
        *byte1 = (uint8_t)((*byte1 & (uint8_t)~high_mask) | (high_val & high_mask));
    }
}

/* ================================================================
 * Minimal PDO/Data structures (document intent, not layout)
 * ================================================================ */
#pragma pack(push,1)
/* 4-DI nibble packed in low 4 bits */
typedef struct {
    uint8_t data_low4;   /* use only bits[3:0] */
} di4_nibble_t;

/* 16-bit output word (e.g., generic module or drive non-motion data) */
typedef struct {
    uint16_t data16;
} out16_t;

/* 20 input bytes (example) */
typedef struct {
    uint8_t bytes[20];
} in20_t;
#pragma pack(pop)

/* ================================================================
 * Integration Layer (TODO)
 * Replace these with your master SDK calls. Keep the signatures.
 * ================================================================ */

/* --- Types that your IL must provide at runtime --- */
typedef struct {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t in_offset_bits;     /* start bit of inputs in PI */
    uint32_t in_size_bits;
    uint32_t out_offset_bits;    /* start bit of outputs in PI */
    uint32_t out_size_bits;
} il_slave_eni_t;

enum { IL_STATE_UNKNOWN=0, IL_STATE_PREOP=1, IL_STATE_SAFEOP=2, IL_STATE_OP=3 };

/* --- Discovery / mapping API (fill these) --- */
static int      il_get_num_slaves(void);                                           /* return N */
static int      il_get_slave_eni(int pos, il_slave_eni_t *out);                    /* 0 ok */
static uint8_t* il_map_output_ptr(int pos, uint32_t byte_offset, uint32_t len);   /* PI out base+offset */
static uint8_t* il_map_input_ptr (int pos, uint32_t byte_offset, uint32_t len);   /* PI in  base+offset */
static int      il_get_state(int pos);                                            /* IL_STATE_* */
static int      il_sdo_read (int pos, uint16_t idx, uint8_t sub, void *buf, int *len); /* 0 ok */
static int      il_sdo_write(int pos, uint16_t idx, uint8_t sub, const void *buf, int len); /* 0 ok */
static void     il_sleep_ms(uint32_t ms);
static void     il_log(const char *fmt, ...);

/* --- Optional SIM backend (no real EtherCAT). Build with IL_ENABLE_SIM=1 to smoke-test the template. --- */
#if IL_ENABLE_SIM
#include <stdarg.h>
static uint8_t  g_sim_pi_in [128];  /* input PI bytes  */
static uint8_t  g_sim_pi_out[128];  /* output PI bytes */

static il_slave_eni_t g_sim_db[] = {
    /* pos 0: EL2004-like (4 DO at bit offset 3) */
    {0x00000002u, 0x07D43052u, 0, 0, 3, 4},
    /* pos 1: EL1004-like (4 DI at bit offset 12) */
    {0x00000002u, 0x03EC3052u, 12, 4, 0, 0},
    /* pos 2: NX-ECC203-like (IN 20B, OUT 16b) at byte boundaries */
    {0x00000083u, 0x000000AAu, 0, 20*8, 0, 16}
};

static int il_get_num_slaves(void){ return (int)(sizeof(g_sim_db)/sizeof(g_sim_db[0])); }
static int il_get_slave_eni(int pos, il_slave_eni_t *out){
    if (pos<0 || pos>=il_get_num_slaves()) return -1;
    *out = g_sim_db[pos]; return 0;
}
static uint8_t* il_map_output_ptr(int pos, uint32_t byte_offset, uint32_t len){
    (void)pos; if (byte_offset+len>sizeof(g_sim_pi_out)) return NULL; return &g_sim_pi_out[byte_offset];
}
static uint8_t* il_map_input_ptr (int pos, uint32_t byte_offset, uint32_t len){
    (void)pos; if (byte_offset+len>sizeof(g_sim_pi_in )) return NULL; return &g_sim_pi_in [byte_offset];
}
static int il_get_state(int pos){ (void)pos; return IL_STATE_OP; }
static int il_sdo_read (int pos, uint16_t idx, uint8_t sub, void *buf, int *len){
    (void)pos;(void)idx;(void)sub; if(len&&*len>=1){((uint8_t*)buf)[0]=0;*len=1;} return 0;
}
static int il_sdo_write(int pos, uint16_t idx, uint8_t sub, const void *buf, int len){
    (void)pos;(void)idx;(void)sub;(void)buf;(void)len; return 0;
}
static void il_sleep_ms(uint32_t ms){ struct timespec ts={ (time_t)(ms/1000u), (long)((ms%1000u)*1000000L)}; nanosleep(&ts,NULL); }
static void il_log(const char *fmt, ...){ va_list ap; va_start(ap,fmt); vprintf(fmt,ap); va_end(ap); }
#else
/* --- Stubs for real integration: compile-time placeholders --- */
static int      il_get_num_slaves(void){ return 0; }
static int      il_get_slave_eni(int pos, il_slave_eni_t *out){ (void)pos;(void)out; return -1; }
static uint8_t* il_map_output_ptr(int pos, uint32_t byte_offset, uint32_t len){ (void)pos;(void)byte_offset;(void)len; return NULL; }
static uint8_t* il_map_input_ptr (int pos, uint32_t byte_offset, uint32_t len){ (void)pos;(void)byte_offset;(void)len; return NULL; }
static int      il_get_state(int pos){ (void)pos; return IL_STATE_UNKNOWN; }
static int      il_sdo_read (int pos, uint16_t idx, uint8_t sub, void *buf, int *len){ (void)pos;(void)idx;(void)sub;(void)buf;(void)len; return -1; }
static int      il_sdo_write(int pos, uint16_t idx, uint8_t sub, const void *buf, int len){ (void)pos;(void)idx;(void)sub;(void)buf;(void)len; return -1; }
static void     il_sleep_ms(uint32_t ms){ (void)ms; }
#include <stdarg.h>
static void     il_log(const char *fmt, ...){ va_list ap; va_start(ap,fmt); vfprintf(stdout,fmt,ap); va_end(ap); }
#endif

/* ================================================================
 * Globals: discovered positions and mapped pointers
 * ================================================================ */
static int  g_pos_do4  = -1;           /* 4xDO nibble module position */
static int  g_pos_di4  = -1;           /* 4xDI nibble module position */
static int  g_pos_out16_in20 = -1;     /* generic 16-bit OUT + 20-byte IN position */

static uint8_t *g_do_b0 = NULL, *g_do_b1 = NULL; /* DO nibble bytes */
static int      g_do_shift = 0;                   /* DO start bit within first byte (0..7) */

static di4_nibble_t *g_di4 = NULL;               /* pointer to DI nibble (byte-aligned view) */

static out16_t *g_out16 = NULL;
static in20_t  *g_in20  = NULL;

/* Bounds (in bits) remembered from ENI */
static int g_di_bits   = 0, g_do_bits = 0;
static int g_in20_bits = 0, g_out16_bits = 0;

/* ================================================================
 * Optional SDO helpers (guarded)
 * ================================================================ */
static void maybe_set_watchdog_ms(int pos, uint16_t idx, uint8_t sub, uint32_t ms) {
#if ENABLE_WATCHDOG_TWEAKS
    /* WHY: Some devices allow extending SM/PDI watchdogs to avoid WDT trips during bring-up.
     * HOW: Read to learn width (16/32-bit), then write matching width.
     * NOTE: Do this in PRE-OP unless vendor doc says otherwise.
     */
    uint8_t buf[4]; int len = (int)sizeof(buf);
    if (il_sdo_read(pos, idx, sub, buf, &len) == 0) {
        if (len == 2) { uint16_t v=(uint16_t)ms; il_sdo_write(pos, idx, sub, &v, 2); }
        else          { uint32_t v=(uint32_t)ms; il_sdo_write(pos, idx, sub, &v, 4); }
    }
#else
    (void)pos; (void)idx; (void)sub; (void)ms;
#endif
}

static void maybe_one_shot_do_poke(int pos, uint16_t idx, uint8_t sub) {
#if ENABLE_ONE_SHOT_SDO_TEST
    uint8_t one=1, zero=0;
    il_sdo_write(pos, idx, sub, &one, 1);
    il_sdo_write(pos, idx, sub, &zero,1);
#else
    (void)pos; (void)idx; (void)sub;
#endif
}

/* ================================================================
 * Init(): detect slaves, map PDOs, optional SDOs (guarded)
 * ================================================================ */
static void Init(void) {
    int n = il_get_num_slaves();
    il_log("Init... slaves=%d\n", n);

    for (int pos = 0; pos < n; ++pos) {
        il_slave_eni_t e; if (il_get_slave_eni(pos, &e) != 0) continue;

        il_log("Slave %d VID=0x%08X PID=0x%08X  in[%u@%u] out[%u@%u]\n",
               pos, e.vendor_id, e.product_code,
               e.in_size_bits, e.in_offset_bits, e.out_size_bits, e.out_offset_bits);

        /* Detect 4xDO nibble by size==4 and non-zero out_size_bits */
        if (g_pos_do4 < 0 && e.out_size_bits == 4) {
            g_pos_do4   = pos;
            g_do_bits   = (int)e.out_size_bits;
            g_do_shift  = (int)(e.out_offset_bits & 7);
            uint32_t byte = e.out_offset_bits / 8u;
            g_do_b0 = il_map_output_ptr(pos, byte, 1);
            g_do_b1 = ( (g_do_shift + 4) > 8 ) ? il_map_output_ptr(pos, byte+1u, 1) : NULL;

            if (!g_do_b0 || ((g_do_shift+4)>8 && !g_do_b1)) {
                il_log("WARN: DO nibble mapping failed (NULL pointer)\n");
                g_pos_do4 = -1;
            } else {
                il_log("DO4 mapped: byte=%u shift=%d bytes=%s\n",
                       (unsigned)byte, g_do_shift, ((g_do_shift+4)>8)?"2":"1");
                maybe_set_watchdog_ms(pos, 0x10F1, 1, 200); /* SM WDT (guarded) */
                maybe_set_watchdog_ms(pos, 0x10F2, 1, 200); /* PDI WDT (guarded) */
                maybe_one_shot_do_poke(pos, 0x7000, 1);     /* demo poke (guarded) */
            }
        }

        /* Detect 4xDI nibble by size==4 and non-zero in_size_bits */
        if (g_pos_di4 < 0 && e.in_size_bits == 4) {
            g_pos_di4 = pos;
            g_di_bits = (int)e.in_size_bits;
            /* For simplicity we request a whole byte at starting byte; we only read low nibble. */
            uint32_t byte = e.in_offset_bits / 8u;
            g_di4 = (di4_nibble_t*)il_map_input_ptr(pos, byte, sizeof(di4_nibble_t));
            if (!g_di4) { il_log("WARN: DI nibble mapping failed\n"); g_pos_di4 = -1; }
        }

        /* Detect "16-bit OUT + 20-byte IN" (example) */
        if (g_pos_out16_in20 < 0 && e.out_size_bits >= 16 && e.in_size_bits >= 20*8) {
            g_pos_out16_in20 = pos;
            g_out16_bits = (int)e.out_size_bits;
            g_in20_bits  = (int)e.in_size_bits;
            g_out16 = (out16_t*)il_map_output_ptr(pos, e.out_offset_bits/8u, sizeof(out16_t));
            g_in20  = (in20_t *)il_map_input_ptr (pos, e.in_offset_bits /8u, sizeof(in20_t));
            if (!g_out16 || !g_in20) { il_log("WARN: OUT16/IN20 mapping failed\n"); g_pos_out16_in20 = -1; }
        }
    }

    /* Sanity tips if nothing mapped */
    if (g_pos_do4<0)  il_log("TIP: No DO4 nibble found — check ENI out_size_bits==4 and offsets.\n");
    if (g_pos_di4<0)  il_log("TIP: No DI4 nibble found — check ENI in_size_bits==4 and offsets.\n");
    if (g_pos_out16_in20<0) il_log("TIP: No OUT16/IN20 module — ok, demo will still run for what was found.\n");
}

/* ================================================================
 * Run(): time-based toggle + input mirror with bounds/state checks
 * ================================================================ */
static void Run(void) {
    static uint64_t next_us = 0;
    static bool     phase = false;    /* OFF/ON */
    static uint16_t flag  = 0;        /* demo flag: mirrors DI into bits and can override OUT16 if bit15=1 */

    /* Timebase decoupled from IO cycle */
    uint64_t t = now_us();
    const uint64_t PERIOD = (uint64_t)BLINK_PERIOD_MS * 1000ULL;

    if (next_us == 0) next_us = t + PERIOD;
    while ((int64_t)(t - next_us) >= 0) { phase = !phase; next_us += PERIOD; }

    /* DO4 nibble: write 0xA/0x5 pattern at mapped bit offset (only in OP) */
    if (g_pos_do4 >= 0 && g_do_b0) {
        if (il_get_state(g_pos_do4) == IL_STATE_OP) {
            write_nibble_at(g_do_b0, g_do_b1, g_do_shift, (uint8_t)(phase ? 0xA : 0x5));
        }
        /* If it "does nothing": verify state==OP, offsets, and that outputs are not masked by safety PLC. */
    }

    /* DI4 nibble: sample and mirror into flag[11:8] (only in OP) */
    if (g_pos_di4 >= 0 && g_di4) {
        if (il_get_state(g_pos_di4) == IL_STATE_OP) {
            uint8_t low4 = (uint8_t)(g_di4->data_low4 & 0x0F);
            flag = (uint16_t)((flag & 0xF0FFu) | ((uint16_t)low4 << 8));
        }
    }

    /* OUT16/IN20 example: write pattern or override by flag, read one input byte into flag[14:12] */
    if (g_pos_out16_in20 >= 0 && g_out16 && g_in20) {
        if (il_get_state(g_pos_out16_in20) == IL_STATE_OP) {
            if (g_out16_bits >= 16) {
                if (flag & 0x8000u) g_out16->data16 = flag;       /* override via bit15 */
                else                g_out16->data16 = (uint16_t)(phase ? 0xAAAAu : 0x5555u);
            }
            if (g_in20_bits >= (19*8)) {
                uint8_t b = g_in20->bytes[18];
                flag = (uint16_t)((flag & 0x8FFFu) | (((uint16_t)b & 0x07u) << 12));
            }
        }
    }

    /* Optional: tiny heartbeat/log each second in SIM */
#if IL_ENABLE_SIM
    static uint64_t last_log=0;
    if (t - last_log > 1000000ULL) {
        il_log("[SIM] phase=%d flag=0x%04X do_b0=0x%02X%s\n",
               phase, flag, g_do_b0?*g_do_b0:0x00, g_do_b1?", split":"");
        last_log = t;
    }
#endif
}

/* ================================================================
 * Simple demo main (build only if you want to run the SIM)
 * In a real app, call Init() once, then Run() each process cycle.
 * ================================================================ */
#ifdef TEMPLATE_BUILD_MAIN
int main(void){
    Init();
    for (int i=0;i<200;i++){ Run(); il_sleep_ms(10); }
    return 0;
}
#endif

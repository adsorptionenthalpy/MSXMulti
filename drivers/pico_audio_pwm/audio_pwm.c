/*
 * pico_audio_pwm - stereo PWM audio output for PICO-56. See audio_pwm.h.
 */

#include "audio_pwm.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#include <string.h>

/* Pins must be a contiguous PWM A/B pair on the same slice. */
#ifndef PWM_AUDIO_PIN_L
#error "PWM_AUDIO_PIN_L must be defined (board config). Even GPIO that's PWM channel A."
#endif
#ifndef PWM_AUDIO_PIN_R
#error "PWM_AUDIO_PIN_R must be defined (board config). PWM_AUDIO_PIN_L + 1."
#endif

/* PWM resolution. 1024 gives ~10 bits of audio per channel.
   PWM frequency = sysclk / PWM_WRAP. At 125 MHz that's ~122 kHz, well above
   audible. */
#define PWM_AUDIO_WRAP 1024

/* Ring buffer. Power-of-two size for cheap masking.
 * 512 samples at 22kHz gives ~23ms buffer. */
#define AUDIO_RING_SIZE 512u
#define AUDIO_RING_MASK (AUDIO_RING_SIZE - 1u)
_Static_assert((AUDIO_RING_SIZE & AUDIO_RING_MASK) == 0,
               "AUDIO_RING_SIZE must be power of two");

static volatile uint32_t s_ring[AUDIO_RING_SIZE];
static volatile uint32_t s_ring_head;  /* write index (producer) */
static volatile uint32_t s_ring_tail;  /* read index  (consumer/IRQ) */

static uint  s_slice;
static int   s_alarm_num   = -1;
static uint32_t s_period_us = 0;
static bool s_running = false;

/* Fractional timing accumulator (16.16 fixed point) for accurate sample rate */
static uint32_t s_period_frac = 0;     /* Period in 16.16 fixed point microseconds */
static uint32_t s_time_accum = 0;      /* Accumulated fractional time */

static inline int16_t sample_left(uint32_t sample_lr)
{
    return (int16_t)(sample_lr >> 16);
}
static inline int16_t sample_right(uint32_t sample_lr)
{
    return (int16_t)(sample_lr & 0xFFFFu);
}

/* Convert signed 16-bit PCM to unsigned PWM level [0, PWM_AUDIO_WRAP]. */
static inline uint16_t pcm_to_pwm(int16_t s)
{
    /* Map -32768..32767 -> 0..PWM_AUDIO_WRAP, centered at PWM_AUDIO_WRAP/2. */
    int32_t v = (int32_t)s + 32768;             /* 0..65535 */
    v = (v * PWM_AUDIO_WRAP) >> 16;             /* 0..PWM_AUDIO_WRAP-1 */
    if (v < 0) v = 0;
    if (v > PWM_AUDIO_WRAP) v = PWM_AUDIO_WRAP;
    return (uint16_t)v;
}

static void __not_in_flash_func(audio_timer_isr)(void)
{
    /* Re-arm the alarm using fractional accumulator for accurate timing.
     * s_period_frac is 16.16 fixed point. We accumulate and use the integer part. */
    if (s_alarm_num >= 0) {
        timer_hw->intr = 1u << s_alarm_num;
        s_time_accum += s_period_frac;
        uint32_t delay_us = s_time_accum >> 16;
        s_time_accum &= 0xFFFF;  /* Keep fractional part */
        timer_hw->alarm[s_alarm_num] = (uint32_t)(timer_hw->timerawl + delay_us);
    }

    uint32_t tail = s_ring_tail;
    uint32_t head = s_ring_head;
    if (tail == head) {
        /* Underrun. Hold last sample (do nothing). */
        return;
    }
    uint32_t lr = s_ring[tail & AUDIO_RING_MASK];
    s_ring_tail = tail + 1u;

#if PWM_AUDIO_LR_SWAPPED
    /* Board routes the lower-numbered GPIO (PWM chan A) to the RIGHT jack
       and the higher to LEFT. Send right sample to chan A. */
    pwm_set_both_levels(s_slice, pcm_to_pwm(sample_right(lr)),
                                  pcm_to_pwm(sample_left(lr)));
#else
    pwm_set_both_levels(s_slice, pcm_to_pwm(sample_left(lr)),
                                  pcm_to_pwm(sample_right(lr)));
#endif
}

bool audio_pwm_setup(uint32_t sample_rate_hz)
{
    if (s_running) return true;
    if (sample_rate_hz == 0) return false;

    /* The two pins must share a PWM slice (A/B). PICO-56 uses 20/21. */
    if ((PWM_AUDIO_PIN_L & 1) != 0) return false;
    if (PWM_AUDIO_PIN_R != PWM_AUDIO_PIN_L + 1) return false;

    gpio_set_function(PWM_AUDIO_PIN_L, GPIO_FUNC_PWM);
    gpio_set_function(PWM_AUDIO_PIN_R, GPIO_FUNC_PWM);
    s_slice = pwm_gpio_to_slice_num(PWM_AUDIO_PIN_L);

    pwm_set_clkdiv_int_frac(s_slice, 1, 0);
    pwm_set_wrap(s_slice, PWM_AUDIO_WRAP);
    pwm_set_both_levels(s_slice, PWM_AUDIO_WRAP / 2, PWM_AUDIO_WRAP / 2);
    pwm_set_enabled(s_slice, true);

    s_ring_head = s_ring_tail = 0;

    /* Use a hardware alarm. Try alarm 3 first; fall back if claimed. */
    s_alarm_num = -1;
    for (int a = 3; a >= 0; --a) {
        if (!hardware_alarm_is_claimed(a)) {
            hardware_alarm_claim(a);
            s_alarm_num = a;
            break;
        }
    }
    if (s_alarm_num < 0) return false;

    /* Calculate period as 16.16 fixed point for accurate timing.
     * s_period_frac = (1000000 << 16) / sample_rate_hz */
    s_period_frac = (uint32_t)(((uint64_t)1000000 << 16) / sample_rate_hz);
    s_period_us = s_period_frac >> 16;  /* Integer part for initial alarm */
    s_time_accum = 0;
    if (s_period_us == 0) s_period_us = 1;

    irq_set_exclusive_handler((TIMER_IRQ_0 + s_alarm_num), audio_timer_isr);
    irq_set_enabled((TIMER_IRQ_0 + s_alarm_num), true);
    timer_hw->inte |= 1u << s_alarm_num;
    timer_hw->alarm[s_alarm_num] = (uint32_t)(timer_hw->timerawl + s_period_us);

    s_running = true;
    return true;
}

void audio_pwm_disable(void)
{
    if (!s_running) return;
    if (s_alarm_num >= 0) {
        timer_hw->inte &= ~(1u << s_alarm_num);
        irq_set_enabled((TIMER_IRQ_0 + s_alarm_num), false);
        hardware_alarm_unclaim(s_alarm_num);
        s_alarm_num = -1;
    }
    pwm_set_enabled(s_slice, false);
    s_running = false;
}

bool audio_pwm_enqueue_sample(uint32_t sample_lr)
{
    uint32_t head = s_ring_head;
    uint32_t next = head + 1u;
    if ((next - s_ring_tail) > AUDIO_RING_SIZE) {
        /* Full: drop. */
        return false;
    }
    s_ring[head & AUDIO_RING_MASK] = sample_lr;
    __sync_synchronize();
    s_ring_head = next;
    return true;
}

uint32_t audio_pwm_writable(void)
{
    uint32_t head = s_ring_head;
    uint32_t tail = s_ring_tail;
    uint32_t used = head - tail;
    if (used >= AUDIO_RING_SIZE) return 0;
    return AUDIO_RING_SIZE - used;
}

bool audio_pwm_dac_error(void) { return false; }
void audio_pwm_mute_internal_speaker(bool mute) { (void)mute; }
void audio_pwm_set_volume(int level)            { (void)level; }
int  audio_pwm_poll_headphone(void)             { return 0; }

void audio_pwm_set_rate(uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0) return;
    s_period_frac = (uint32_t)(((uint64_t)1000000 << 16) / sample_rate_hz);
    s_period_us = s_period_frac >> 16;
    s_time_accum = 0;  /* Reset accumulator on rate change */
    if (s_period_us == 0) s_period_us = 1;
}

uint32_t audio_pwm_get_rate(void)
{
    if (s_period_frac == 0) return 0;
    return (uint32_t)(((uint64_t)1000000 << 16) / s_period_frac);
}

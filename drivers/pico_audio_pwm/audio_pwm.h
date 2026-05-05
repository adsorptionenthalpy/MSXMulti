/*
 * pico_audio_pwm - stereo PWM audio output for PICO-56
 *
 * Outputs signed 16-bit stereo PCM as a pair of PWM channels driving
 * GPIO pins (passed through an RC low-pass on the board to two RCA
 * jacks). PWM_AUDIO_PIN_L must map to PWM channel A of some slice and
 * PWM_AUDIO_PIN_R must be the matching channel B of the same slice
 * (i.e. PIN_R = PIN_L + 1 with PIN_L even). PICO-56 uses GPIO 20/21
 * which satisfies that.
 *
 * Sample feeding: a hardware timer ISR fires at audio_pwm_setup()'s
 * sample rate and pulls one stereo sample from a small ring buffer.
 * audio_pwm_enqueue_sample() pushes 16-bit signed L+R packed into a
 * single uint32 (high half = L, low half = R), matching the
 * EXT_AUDIO_ENQUEUE_SAMPLE() convention used by audio_i2s.
 *
 * If the ring is full, the oldest sample is dropped (better than
 * stalling the emulator).
 */

#ifndef PICO_AUDIO_PWM_H
#define PICO_AUDIO_PWM_H

#include <inttypes.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize PWM audio. sample_rate_hz typical 22050 for NES emulation.
   Returns true on success. */
bool audio_pwm_setup(uint32_t sample_rate_hz);

/* Stop and release PWM/timer. */
void audio_pwm_disable(void);

/* Enqueue one stereo sample. Packed as (L<<16)|(R) where L/R are
   16-bit signed PCM samples (sign-extended). Same packing used by
   audio_i2s_enqueue_sample(). Returns true if accepted, false if the
   ring overflowed. */
bool audio_pwm_enqueue_sample(uint32_t sample_lr);

/* Number of free slots in the ring buffer (in stereo samples). */
uint32_t audio_pwm_writable(void);

/* Returns false: PWM has no DAC to fail. Provided for parity with
   the audio_i2s API. */
bool audio_pwm_dac_error(void);

/* No internal speaker / volume control on PICO-56. Provided for
   parity with the audio_i2s macro layer. */
void audio_pwm_mute_internal_speaker(bool mute);
void audio_pwm_set_volume(int level);
int  audio_pwm_poll_headphone(void);

/* Change the output sample rate on the fly. */
void audio_pwm_set_rate(uint32_t sample_rate_hz);
uint32_t audio_pwm_get_rate(void);

#ifdef __cplusplus
}
#endif

#endif /* PICO_AUDIO_PWM_H */

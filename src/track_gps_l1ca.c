/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_gps_l1ca.h"
#include "track_api.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>

#include "settings.h"
#include <inttypes.h>

#define NUM_GPS_L1CA_TRACKERS   12

/** Total count of loop parameters per row */
#define LOOP_PARAMS_COUNT (11)

/*  code: nbw zeta k carr_to_code
 carrier:                    nbw  zeta k fll_aid
 pipelining:                                   type k */
#define LOOP_PARAMS_SLOW \
  "(1 ms, (1, 0.7, 1, 1540), (10, 0.7, 1, 5), (1, 0.95) )," \
 "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0), (2, 0.95) )"

#define LOOP_PARAMS_10MS \
  "(1 ms, (1, 0.7, 1, 1540), (10, 0.7, 1, 5), (0, 0.95) )," \
 "(10 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0), (1, 0.95) )"

#define LOOP_PARAMS_MED \
  "(1 ms, (1, 0.7, 1, 1540), (10, 0.7, 1, 5), (0, 0.0) )," \
  "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0), (1, 0.8) )"

#define LOOP_PARAMS_FAST \
  "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5), (1, 0.95) )," \
  "(4 ms, (1, 0.7, 1, 1540), (62, 0.7, 1, 0), (1, 0.95) )"

#define LOOP_PARAMS_EXTRAFAST \
  "(1 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 5), (1, 0.0) )," \
  "(2 ms, (1, 0.7, 1, 1540), (100, 0.7, 1, 0), (1, 0.95) )"

/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS_PESS     "0.10, 1.4, 200, 50"
#define LD_PARAMS_NORMAL   "0.05, 1.4, 150, 50"
#define LD_PARAMS_OPT      "0.02, 1.1, 150, 50"
#define LD_PARAMS_EXTRAOPT "0.02, 0.8, 150, 50"
#define LD_PARAMS_DISABLE  "0.02, 1e-6, 1, 1"

#define CN0_EST_LPF_CUTOFF 5
#define CN0_EST_LPF_ALPHA 0.1f

/** No predictions, strict 1ms integration. */
#define LOOP_MODE_INITIAL      (0u)
/** Predictions, normal integration. */
#define LOOP_MODE_NORMAL       (1u)

static struct loop_params {
  float code_bw, code_zeta, code_k, carr_to_code;
  float carr_bw, carr_zeta, carr_k, carr_fll_aid_gain;
  float predict_k;
  u8 coherent_ms;
  u8 mode;
} loop_params_stage[2];

static struct lock_detect_params {
  float k1, k2;
  u16 lp, lo;
} lock_detect_params;

static float track_cn0_use_thres = 31.0; /* dBHz */
static float track_cn0_drop_thres = 31.0;

static char loop_params_string[120] = LOOP_PARAMS_MED;
static char lock_detect_params_string[24] = LD_PARAMS_DISABLE;
static bool use_alias_detection = true;

typedef struct {
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  u32 code_phase_rate_fp;      /**< Code phase rate in NAP register units. */
  u32 code_phase_rate_fp_prev; /**< Previous code phase rate in NAP register units. */
  s32 carrier_freq_fp;         /**< Carrier frequency in NAP register units. */
  s32 carrier_freq_fp_prev;    /**< Previous carrier frequency in NAP register units. */
  u32 corr_sample_count;       /**< Number of samples in correlation period. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  u8 stage;                    /**< 0 = First-stage. 1 ms integration.
                                    1 = Second-stage. After nav bit sync,
                                    retune loop filters and typically (but
                                    not necessarily) use longer integration. */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */

  bool first;
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  double code_phase_rate_prev; /**< Code phase rate in chips/s. */
  double carrier_freq;         /**< Carrier frequency Hz. */
  double carrier_freq_prev;    /**< Carrier frequency Hz. */

  const struct loop_params *stage_params[3]; /**< Loop filter parameters: measured, current, future */
} gps_l1ca_tracker_data_t;

static tracker_t gps_l1ca_trackers[NUM_GPS_L1CA_TRACKERS];
static gps_l1ca_tracker_data_t gps_l1ca_tracker_data[NUM_GPS_L1CA_TRACKERS];

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static bool parse_loop_params(struct setting *s, const char *val);
static bool parse_lock_detect_params(struct setting *s, const char *val);

static const tracker_interface_t tracker_interface_gps_l1ca = {
  .code =         CODE_GPS_L1CA,
  .init =         tracker_gps_l1ca_init,
  .disable =      tracker_gps_l1ca_disable,
  .update =       tracker_gps_l1ca_update,
  .trackers =     gps_l1ca_trackers,
  .num_trackers = NUM_GPS_L1CA_TRACKERS
};

static tracker_interface_list_element_t
tracker_interface_list_element_gps_l1ca = {
  .interface = &tracker_interface_gps_l1ca,
  .next = 0
};

void track_gps_l1ca_register(void)
{
  SETTING_NOTIFY("track", "loop_params", loop_params_string,
                 TYPE_STRING, parse_loop_params);
  SETTING_NOTIFY("track", "lock_detect_params", lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params);
  SETTING("track", "cn0_use", track_cn0_use_thres, TYPE_FLOAT);
  SETTING("track", "cn0_drop", track_cn0_drop_thres, TYPE_FLOAT);
  SETTING("track", "alias_detect", use_alias_detection, TYPE_BOOL);

  for (u32 i=0; i<NUM_GPS_L1CA_TRACKERS; i++) {
    gps_l1ca_trackers[i].active = false;
    gps_l1ca_trackers[i].data = &gps_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
}

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l1ca_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l1ca_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  const struct loop_params *l = &loop_params_stage[0];
  data->stage_params[0] = data->stage_params[1] = l;
  data->stage_params[2] = NULL; /* no change planned */

  /* Note: The only coherent integration interval currently supported
     for first-stage tracking (i.e. loop_params_stage[0].coherent_ms)
     is 1. */

  aided_tl_init(&(data->tl_state), 1e3 / l->coherent_ms,
                common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                common_data->carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->carr_fll_aid_gain);
  data->first = true;

  data->code_phase_rate = common_data->code_phase_rate;
  data->code_phase_rate_fp = common_data->code_phase_rate*NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  data->code_phase_rate_fp_prev = data->code_phase_rate_fp;
  data->carrier_freq = common_data->carrier_freq;
  data->carrier_freq_fp = (s32)(common_data->carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  data->carrier_freq_fp_prev = data->carrier_freq_fp;

  /* Initialize C/N0 estimator */
  cn0_est_init(&data->cn0_est, 1e3/l->coherent_ms, common_data->cn0, CN0_EST_LPF_CUTOFF, 1e3/l->coherent_ms);
  common_data->cn0_lpf = common_data->cn0;

  lock_detect_init(&data->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);

  /* TODO: Reconfigure alias detection between stages */
  u8 alias_detect_ms = MIN(loop_params_stage[1].coherent_ms,
                           tracker_bit_length_get(channel_info->context));
  alias_detect_init(&data->alias_detect, 500/alias_detect_ms,
                    (alias_detect_ms-1)*1e-3);

}

static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)channel_info;
  (void)common_data;
  (void)tracker_data;
}

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  char buf[SID_STR_LEN_MAX];
  sid_to_string(buf, sizeof(buf), channel_info->sid);

  /* Number of milliseconds in the integrated period */
  u8 coherent_ms = data->stage_params[0]->coherent_ms;

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
    tracker_correlations_read(channel_info->context, data->cs,
                              &data->corr_sample_count);
    alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);

  common_data->sample_count += data->corr_sample_count;
  common_data->code_phase_early = (u64)common_data->code_phase_early +
                           (u64)data->corr_sample_count
                             * data->code_phase_rate_fp_prev;
  common_data->carrier_phase += (s64)data->carrier_freq_fp_prev
                           * data->corr_sample_count;
  /* TODO: Fix this in the FPGA - first integration is one sample short. */
  if (common_data->update_count == 0) {
    common_data->carrier_phase -= data->carrier_freq_fp_prev;
    common_data->report_count = 0;
  }
  data->code_phase_rate_fp_prev = data->code_phase_rate_fp;
  data->carrier_freq_fp_prev = data->carrier_freq_fp;

  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           coherent_ms);
  common_data->update_count += coherent_ms;
  tracker_bit_sync_update(channel_info->context, coherent_ms, data->cs[1].I);

  /* Correlations should already be in chan->cs thanks to
   * tracking_channel_get_corrs. */
  corr_t* cs = data->cs;

  /* Update C/N0 estimate */
  common_data->cn0 = cn0_est(&data->cn0_est,
                             (float)cs[1].I / coherent_ms,
                             (float)cs[1].Q / coherent_ms);

  /* C/N0 low pass filter for console reporting */
  common_data->cn0_lpf = CN0_EST_LPF_ALPHA * common_data->cn0 +
      (1.f - CN0_EST_LPF_ALPHA) * common_data->cn0_lpf;

  if (common_data->cn0 > track_cn0_drop_thres)
    common_data->cn0_above_drop_thres_count = common_data->update_count;

  if (common_data->cn0 < track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, cs[1].I, cs[1].Q, coherent_ms);

  if (data->lock_detect.outo) {
    common_data->ld_opti_locked_count = common_data->update_count;
  }
  if (!data->lock_detect.outp) {
    common_data->ld_pess_unlocked_count = common_data->update_count;
  }

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp) {
    if (data->stage > 0) {
      log_info("%s PLL stress", buf);
    }
    tracker_ambiguity_unknown(channel_info->context);
  }

  /* Run the loop filters. */

  /* TODO: Make this more elegant. */
  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = cs[2-i].I;
    cs2[i].Q = cs[2-i].Q;
  }

  /* Output I/Q correlations using SBP if enabled for this channel */
  if (coherent_ms > 1) {
    tracker_correlations_send(channel_info->context, cs);
  }
  if (data->first) {
    data->first = false;
    data->tl_state.prev_I = 1.0f; // This works, but is it a really good way to do it?
    data->tl_state.prev_Q = 0.0f;
  } else {
    aided_tl_update(&data->tl_state, cs2);
  }

  /* ------------------------------------------------------------------
   * Carrier and code frequencies
   * ------------------------------------------------------------------ */
  /* Z{0} */
  common_data->carrier_freq_prev = common_data->carrier_freq;
  common_data->code_phase_rate_prev = common_data->code_phase_rate;

  /* Z{-1} */
  common_data->carrier_freq = data->tl_state.carr_freq;
  common_data->code_phase_rate = data->tl_state.code_freq + GPS_CA_CHIPPING_RATE;

  if (common_data->update_count != 0) {
    /* There is an error between target frequency and actual one. Affect
     * the target frequency according to the computed error */
    double pipelining_k = data->stage_params[1]->predict_k;

    if (0. != pipelining_k) {
      double carr_freq_error = common_data->carrier_freq -
                               common_data->carrier_freq_prev;
      common_data->carrier_freq += carr_freq_error * pipelining_k;

      double code_freq_error = common_data->code_phase_rate -
                               common_data->code_phase_rate_prev;
      common_data->code_phase_rate += code_freq_error * pipelining_k;
    }
  }

  data->code_phase_rate_fp_prev = data->code_phase_rate_fp;
  data->code_phase_rate_fp = common_data->code_phase_rate
    * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

  data->carrier_freq_fp = common_data->carrier_freq
    * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;

  /* Attempt alias detection if we have pessimistic phase lock detect, OR
     (optimistic phase lock detect AND are in second-stage tracking) */
  if (use_alias_detection &&
      (data->lock_detect.outp ||
       (data->lock_detect.outo && data->stage > 0))) {

    s32 I = (cs[1].I - data->alias_detect.first_I) / coherent_ms;
    s32 Q = (cs[1].Q - data->alias_detect.first_Q) / coherent_ms;
    float err = alias_detect_second(&data->alias_detect, I, Q);
    if (fabs(err) > (250.f / coherent_ms)) {
      if (data->lock_detect.outp) {
        log_warn("False phase lock detected on %s: err=%f", buf, err);
      }

      tracker_ambiguity_unknown(channel_info->context);
      /* Indicate that a mode change has occurred. */
      common_data->mode_change_count = common_data->update_count;

      data->tl_state.carr_freq += err;
      data->tl_state.carr_filt.y = data->tl_state.carr_freq;
    }
  }

  /* ------------------------------------------------------------------
   * Operational mode control: mode selection
   * ------------------------------------------------------------------ */
  /* Stage transitions: consider stage transitions only on bit sync. */

  /* Consider moving from stage 0 (1 ms integration) to stage 1 (longer). */
  bool nextBitSync = tracker_next_bit_aligned(channel_info->context,
                                           data->stage_params[1]->coherent_ms);

  /* ------------------------------------------------------------------
   * Stage transitions: consider stage transitions only on bit sync
   * ------------------------------------------------------------------ */
  const struct loop_params *l  = NULL; /* Next parameters */
  if (nextBitSync) {
    /* This integration interval is the last one within the bit boundary. It
     * can also be the only interval (in case of 20ms integrations). */

    if (/* The initial stage is in progress */
        0 == data->stage &&
        /* Must have (at least optimistic) phase lock */
        data->lock_detect.outo) {
      log_info("%s synced @ %u ms, %.1f (LPF %.1f) dBHz",
               buf, (unsigned int)common_data->update_count,
               common_data->cn0,
               common_data->cn0_lpf);
      data->stage = 1;
      l = &loop_params_stage[data->stage];
    } else {
      if (common_data->update_count - common_data->report_count >= 10000) {
        common_data->report_count = common_data->update_count;
        log_info("%s continue @ %u ms, %.1f (LPF %.1f) dBHz",
                 buf, (unsigned int)common_data->update_count,
                 common_data->cn0,
                 common_data->cn0_lpf);
      }
    }
  }
  /* ------------------------------------------------------------------
   * Operational mode control: mode change in effect
   * ------------------------------------------------------------------ */

  if (data->stage_params[1] != data->stage_params[0]) {
    /* Pipelining: the signal processing is almost done. Next integration
     * period would be updated according to the currently pending
     * configuration.
     */
    const struct loop_params *old = data->stage_params[0];
    const struct loop_params *cur = data->stage_params[0] = data->stage_params[1];

    if (old->coherent_ms != cur->coherent_ms) {
      cn0_est_init(&data->cn0_est, 1e3 / cur->coherent_ms, common_data->cn0,
                   CN0_EST_LPF_CUTOFF, 1e3 / cur->coherent_ms);

      lock_detect_reinit(&data->lock_detect,
                         lock_detect_params.k1 * cur->coherent_ms,
                         lock_detect_params.k2,
                         /* TODO: Should also adjust lp and lo? */
                         lock_detect_params.lp, lock_detect_params.lo);
    }
    /* Mode switch:
     * - Default mode. Pipelining corrections are not used.
     * - Normal mode. Normal pipelining with corrections.
     * - One plus N. Integration interval is split into 1ms plus the rest.
     *   This mode is not feasible when integration time is less than 3.
     */
    if (old->coherent_ms != cur->coherent_ms ||
        old->code_bw != cur->code_bw ||
        old->code_zeta != cur->code_zeta ||
        old->code_k != cur->code_k ||
        old->carr_bw != cur->carr_bw ||
        old->carr_zeta != cur->carr_zeta ||
        old->carr_k != cur->carr_k ||
        old->carr_fll_aid_gain != cur->carr_fll_aid_gain) {

      /* Recalculate filter coefficients */
      aided_tl_retune(&data->tl_state, 1e3 / cur->coherent_ms,
                      cur->code_bw, cur->code_zeta, cur->code_k,
                      cur->carr_to_code,
                      cur->carr_bw, cur->carr_zeta, cur->carr_k,
                      cur->carr_fll_aid_gain);
      data->first = true;
    }
  }

  /* ------------------------------------------------------------------
   * Operational mode control: mode activation
   * ------------------------------------------------------------------ */

  if (NULL != l && data->stage_params[1] != l) {
    log_info("%s: scheduling integration time %" PRIu8 " ms",
             buf, l->coherent_ms);

    /* Pipelining: next integration time can't be changed. So the change
     * takes effect after the next interval. */
    data->stage_params[1] = l;

    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;
  }

  tracker_retune(channel_info->context,
                 data->carrier_freq_fp,
                 data->code_phase_rate_fp,
                 data->stage_params[1]->coherent_ms - 1);
}

/** Parse a string describing the tracking loop filter parameters into
    the loop_params_stage structs. */
static bool parse_loop_params(struct setting *s, const char *val)
{
  /** The string contains loop parameters for either one or two
      stages.  If the second is omitted, we'll use the same parameters
      as the first stage.*/

  struct loop_params loop_params_parse[2];

  const char *str = val;
  for (int stage = 0; stage < 2; stage++) {
    struct loop_params *l = &loop_params_parse[stage];

    int n_chars_read = 0;
    u16 tmp, tmp2;

    if (sscanf(str,
               "( %" SCNu16 " ms , "       /* Coherent integration time ms */
               "( %f , %f , %f , %f ) , " /* Code tracker parameters */
               "( %f , %f , %f , %f ) , " /* Carrier tracker parameters */
               "( %" SCNu16 " , %f ) ) , " /* Pipelining parameters */
               "%n",                      /* Total number of characters */
               &tmp,
               &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
               &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->carr_fll_aid_gain,
               &tmp2, &l->predict_k,
               &n_chars_read) < LOOP_PARAMS_COUNT) {
      log_error("Ill-formatted tracking loop param string.");
      return false;
    }
    l->coherent_ms = tmp;
    l->mode = tmp2;
    /* If string omits second-stage parameters, then after the first
       stage has been parsed, n_chars_read == 0 because of missing
       comma and we'll parse the string again into loop_params_parse[1]. */
    str += n_chars_read;

    log_info("Stage %d ms=%"PRIu8" mode=%"PRIu8" K=%lf", stage, l->coherent_ms, l->mode, l->predict_k);

    if ((l->coherent_ms == 0)
        || ((20 % l->coherent_ms) != 0) /* i.e. not 1, 2, 4, 5, 10 or 20 */
        || (stage == 0 && l->coherent_ms != 1)) {
      log_error("Invalid coherent integration length.");
      return false;
    }
  }
  /* Successfully parsed both stages.  Save to memory. */
  strncpy(s->addr, val, s->len);
  memcpy(loop_params_stage, loop_params_parse, sizeof(loop_params_stage));
  return true;
}

/** Parse a string describing the tracking loop phase lock detector
    parameters into the lock_detect_params structs. */
static bool parse_lock_detect_params(struct setting *s, const char *val)
{
  struct lock_detect_params p;

  if (sscanf(val, "%f , %f , %" SCNu16 " , %" SCNu16,
             &p.k1, &p.k2, &p.lp, &p.lo) < 4) {
      log_error("Ill-formatted lock detect param string.");
      return false;
  }
  /* Successfully parsed.  Save to memory. */
  strncpy(s->addr, val, s->len);
  memcpy(&lock_detect_params, &p, sizeof(lock_detect_params));
  return true;
}

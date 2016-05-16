/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_profiles.h"
#include "chconf_board.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>

#include <board.h>
#include <platform_signal.h>

#include <string.h>

#define TP_USE_1MS_PROFILES
#define TP_USE_2MS_PROFILES
// #define TP_USE_5MS_PROFILES
#define TP_USE_10MS_PROFILES
#define TP_USE_20MS_PROFILES

#define TP_MAX_SUPPORTED_SVS NUM_GPS_L1CA_TRACKERS
#define ARR_SIZE(x) (sizeof(x)/sizeof(x[0]))

#define TP_DEFAULT_CN0_USE_THRESHOLD 31.f
#define TP_DEFAULT_CN0_DROP_THRESHOLD 31.f
#define TP_DEFAULT_CN0_ITIME_UPGRADE_MARGIN 2.f

#define TP_SNR_THRESHOLD_MIN (TP_DEFAULT_CN0_USE_THRESHOLD + \
                              TP_DEFAULT_CN0_ITIME_UPGRADE_MARGIN)
#define TP_SNR_THRESHOLD_MAX  36.f
#define TP_SNR_THRESHOLD_LOCK 31
#define TP_SNR_CHANGE_LOCK_MS 1250


/**
 * Per-satellite entry.
 */
typedef struct {
  bool          used;
  gnss_signal_t sid;
  tp_report_t   last_report;  /**< Last data from tracker */
  /** @todo TODO LP filter for CN0 */
  /** @todo TODO LP filter for speed/acceleration */
  u32           profile_update:1;  /**< Flag if the profile update is required */
  u32           profile_in_queue:1;  /**< Flag if the profile update is required */
  u32           cur_profile_i:3;   /**< Index of the currently active profile */
  u32           cur_profile_d:3;   /**< Index of the currently active profile */
  u32           next_profile_i:3;  /**< Index of the next selected profile */
  u32           next_profile_d:3;  /**< Index of the next selected profile */
  u32           low_cn0_count:5;
  u32           high_cn0_count:5;
  u16           lock_time_ms:16; /**< Lock time countdown timer */
  float         prev_val[4];  /**< Filtered counters */
  float         filt_val[4];  /**< Filtered counters */
  float         mean_acc[4];  /**< Mean accumulators */
  u32           mean_cnt;     /**< Mean value divider */
  lp1_filter_t  speed_filter;  /**< Value filter */
  lp1_filter_t  accel_filter;  /**< Value filter */
  lp1_filter_t  jitter_filter; /**< Value filter */
  lp1_filter_t  cn0_filter;    /**< Value filter */
  u32           time;          /**< Tracking time */
  u32           last_print_time; /**< Last debug print time */
} tp_profile_internal_t;

/**
 * GPS satellite profiles.
 */
static tp_profile_internal_t profiles_gps1[TP_MAX_SUPPORTED_SVS] _CCM;

/**
 * C/N0 profile
 */
static const tp_cn0_params_t cn0_params_default = {
  .track_cn0_drop_thres = TP_DEFAULT_CN0_DROP_THRESHOLD,
  .track_cn0_use_thres = TP_DEFAULT_CN0_USE_THRESHOLD
};

/**
 * Lock detector profile
 */
static const tp_lock_detect_params_t ld_params_disable = {
  .k1 = 0.02f,
  .k2 = 1e-6f,
  .lp = 1,
  .lo = 1
};

/**
 * Initial tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_initial = {
  // "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5)),
  .coherent_ms = 1,
  .carr_bw = 50,
  .carr_zeta = .5f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 2,
  .code_zeta = 0.5f,
  .code_k = 1,
  .carr_fll_aid_gain = 5,
  .mode = TP_TM_PIPELINING
};

#ifdef TP_USE_1MS_PROFILES
/**
 * Initial tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_1ms = {
  // "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5)),
  .coherent_ms = 1,
  .carr_bw = 7,
  .carr_zeta = 2.f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 2.f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_PIPELINING
};
#endif /* TP_USE_1MS_PROFILES */
#ifdef TP_USE_2MS_PROFILES

/**
 * 2 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_2ms = {
  // "(2 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))"

  .coherent_ms = 2,
  .carr_bw = 14,
  .carr_zeta = 1.f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 1.f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_PIPELINING
};
#endif /* TP_USE_2MS_PROFILES */
#ifdef TP_USE_5MS_PROFILES

/**
 * 5 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_5ms = {
  // "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))"

  .coherent_ms = 5,
  .carr_bw = 50,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
#endif /* TP_USE_5MS_PROFILES */
#ifdef TP_USE_10MS_PROFILES
/**
 * 20 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_10ms = {
  //  "(10 ms, (1, 0.7, 1, 1540), (30, 0.7, 1, 0))"

  .coherent_ms = 10,
  .carr_bw = 25,
  .carr_zeta = .7f,
  .carr_k = 1.,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = .7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
#endif /* TP_USE_10MS_PROFILES */
#ifdef TP_USE_20MS_PROFILES
/**
 * 20 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_20ms = {
  //  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))"

  .coherent_ms = 20,
  .carr_bw = 5, // 10/.9 is good; 5(1. is better
  .carr_zeta = .9f,
  .carr_k = 2.f,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = .9f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
#endif /* TP_USE_20MS_PROFILES */

enum
{
  TP_PROFILE_INI=0,
#ifdef TP_USE_1MS_PROFILES
  TP_PROFILE_1MS,
#endif
#ifdef TP_USE_2MS_PROFILES
  TP_PROFILE_2MS,
#endif
#ifdef TP_USE_5MS_PROFILES
  TP_PROFILE_5MS,
#endif
#ifdef TP_USE_10MS_PROFILES
  TP_PROFILE_10MS,
#endif
#ifdef TP_USE_20MS_PROFILES
  TP_PROFILE_20MS,
#endif
  TP_PROFILE_TIME_COUNT,
  TP_PROFILE_TIME_FIRST = 1
};

enum
{
  TP_PROFILE_SLOW = 0,
  TP_PROFILE_MED,
  TP_PROFILE_FAST,
  TP_PROFILE_SPEED_COUNT,
  TP_PROFILE_SPEED_FIRST = 0
};

enum
{
  TP_LP_IDX_INI,
#ifdef TP_USE_1MS_PROFILES
  TP_LP_IDX_1MS,
#endif
#ifdef TP_USE_2MS_PROFILES
  TP_LP_IDX_2MS,
#endif
#ifdef TP_USE_5MS_PROFILES
  TP_LP_IDX_5MS,
#endif
#ifdef TP_USE_10MS_PROFILES
  TP_LP_IDX_10MS,
#endif
#ifdef TP_USE_20MS_PROFILES
  TP_LP_IDX_20MS
#endif
};

static const tp_loop_params_t *loop_params[] = {
  &loop_params_initial,
#ifdef TP_USE_1MS_PROFILES
  &loop_params_1ms,
#endif
#ifdef TP_USE_2MS_PROFILES
  &loop_params_2ms,
#endif
#ifdef TP_USE_5MS_PROFILES
  &loop_params_5ms,
#endif
#ifdef TP_USE_10MS_PROFILES
  &loop_params_10ms,
#endif
#ifdef TP_USE_20MS_PROFILES
  &loop_params_20ms
#endif
};

/**
 * State transition matrix.
 *
 * Matrix is two-dimensional: first dimension enumerates integration times,
 * second dimension is the dynamics profile.
 *
 */
static const u8 profile_matrix[][TP_PROFILE_SPEED_COUNT] = {
  {TP_LP_IDX_INI,  TP_LP_IDX_INI,  TP_LP_IDX_INI},
#ifdef TP_USE_1MS_PROFILES
  {TP_LP_IDX_1MS,  TP_LP_IDX_1MS,  TP_LP_IDX_1MS},
#endif
#ifdef TP_USE_2MS_PROFILES
  {TP_LP_IDX_2MS,  TP_LP_IDX_2MS,  TP_LP_IDX_2MS},
#endif
#ifdef TP_USE_5MS_PROFILES
  {TP_LP_IDX_5MS,  TP_LP_IDX_5MS,  TP_LP_IDX_5MS},
#endif
#ifdef TP_USE_10MS_PROFILES
  {TP_LP_IDX_10MS, TP_LP_IDX_10MS, TP_LP_IDX_10MS},
#endif
#ifdef TP_USE_20MS_PROFILES
  {TP_LP_IDX_20MS, TP_LP_IDX_20MS, TP_LP_IDX_20MS}
#endif
};

/**
 * Helper method for computing GNSS satellite speed from doppler.
 *
 * The method converts doppler frequency shift relative vector speed towards
 * the line of sight.
 *
 * \params[in] sid  GNSS satellite signal identifier.
 * \params[in] data Sattelite tracking loop report data.
 *
 * \returns Speed in meters per second, or 0. on error.
 */
static double compute_speed(gnss_signal_t sid, const tp_report_t *data)
{
  double speed_mps = 0.;
  double doppler_hz = data->carr_freq; /* Carrier frequency is actually a
                                        * doppler frequency shift */

  switch (sid.code) {
  case CODE_GPS_L1CA:
    speed_mps = -(double)GPS_L1_LAMBDA_NO_VAC * doppler_hz;
    break;
  case CODE_GPS_L2CM:
  default:
    /* Do not support */
    break;
  }

  return speed_mps;
}

/**
 * Allocates a new tracking profile structure for a satellite.
 *
 * Profiles identify physical GNSS satellites, not their signals.
 *
 * \param[in] sid  GNSS satellite signal identifier.
 *
 * \return Allocated profile pointer.
 * \retval NULL on error.
 */
static tp_profile_internal_t *allocate_profile(gnss_signal_t sid)
{
  size_t i;
  tp_profile_internal_t *res = NULL;

  for (i = 0; i< TP_MAX_SUPPORTED_SVS; ++i) {
    if (!profiles_gps1[i].used) {
      res = &profiles_gps1[i];
      break;
    }
  }
  if (NULL != res) {
    res->used = true;
    res->sid = sid;
  }
  return res;
}

static tp_profile_internal_t *find_profile(gnss_signal_t sid)
{
  size_t i;
  tp_profile_internal_t *res = NULL;

  for (i = 0; i< TP_MAX_SUPPORTED_SVS; ++i) {
    if (profiles_gps1[i].used &&
        profiles_gps1[i].sid.code == sid.code &&
        profiles_gps1[i].sid.sat == sid.sat) {
      res = &profiles_gps1[i];
      break;
    }
  }
  return res;
}

static void delete_profile(gnss_signal_t sid)
{
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != profile) {
    memset(profile, 0, sizeof(*profile));
  }
}

static void get_profile_params(gnss_signal_t sid,
                               u8            profile_i,
                               u8            profile_d,
                               tp_config_t  *config)
{
  u8 profile = profile_matrix[profile_i][profile_d];

  log_debug_sid(sid, "Activating profile %u [%u][%u])",
                profile, profile_i, profile_d);

  config->lock_detect_params = ld_params_disable;
  config->loop_params = *loop_params[profile];
  config->use_alias_detection = false;
  config->cn0_params = cn0_params_default;

  if (config->loop_params.coherent_ms > 1) {
    /* Denormalize C/N0. */
    float cn0_delta = 0.f;
    switch (config->loop_params.mode) {
    case TP_TM_ONE_PLUS_N:
      cn0_delta = 10.f * log10f(config->loop_params.coherent_ms - 1);
      break;
    case TP_TM_PIPELINING:
    case TP_TM_IMMEDIATE:
    default:
      cn0_delta = 10.f * log10f(config->loop_params.coherent_ms);
      break;
    }
    log_info_sid(sid, "CN0 offset %f @ %d", cn0_delta, config->loop_params.coherent_ms);

    config->cn0_params.track_cn0_drop_thres -= cn0_delta;
    if (config->cn0_params.track_cn0_drop_thres < 23.) {
      config->cn0_params.track_cn0_drop_thres = 23.;
    }
    config->cn0_params.track_cn0_use_thres -= cn0_delta;
    if (config->cn0_params.track_cn0_drop_thres < 23.) {
      config->cn0_params.track_cn0_use_thres = 23.;
    }
  }
}

static void update_stats(gnss_signal_t sid,
                         tp_profile_internal_t *profile,
                         const tp_report_t *data)
{
  float dt_sec = 0.001f * data->time_ms;
  float speed, accel, jitter, cn0;

  speed = compute_speed(sid, data);
  accel = (profile->prev_val[0] - speed) / dt_sec;
  jitter = (profile->prev_val[1] - accel) / dt_sec;
  cn0 = data->cn0;

  profile->mean_acc[0] += speed * speed;
  profile->mean_acc[1] += accel * accel;
  profile->mean_acc[2] += jitter * jitter;
  profile->mean_acc[3] += cn0 * cn0;
  profile->mean_cnt += 1;

  profile->prev_val[0] = speed;
  profile->prev_val[1] = accel;
  profile->prev_val[2] = jitter;
  profile->prev_val[3] = cn0;

  speed = lp1_filter_update(&profile->speed_filter, speed);
  accel = (profile->filt_val[0] - speed) / dt_sec;
  jitter = (profile->filt_val[1] - accel) / dt_sec;
//  accel = lp1_filter_update(&profile->accel_filter, accel);
//  jitter = lp1_filter_update(&profile->jitter_filter, jitter);
  cn0 = lp1_filter_update(&profile->cn0_filter, data->cn0);

  profile->filt_val[0] = speed;
  profile->filt_val[1] = accel;
  profile->filt_val[2] = jitter;
  profile->filt_val[3] = cn0;

}

static void print_stats(gnss_signal_t sid, tp_profile_internal_t *profile)
{
  if (profile->time - profile->last_print_time >= 20000) {
    profile->last_print_time = profile->time;

    float div = 1.f;
    if (profile->mean_cnt > 0)
      div = 1.f / profile->mean_cnt;

    float s = sqrtf(profile->mean_acc[0] * div);
    float a = sqrtf(profile->mean_acc[1] * div);
    float j = sqrtf(profile->mean_acc[2] * div);
    float c = sqrtf(profile->mean_acc[3] * div);

    u8 lp_idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];
    log_info_sid(sid,
                 "MV: T=%dms N=%d CN0=%.2f s=%.3f a=%.3f j=%.3f",
                 (int)loop_params[lp_idx]->coherent_ms,
                 profile->mean_cnt,
                 c, s, a, j
                );
    log_info_sid(sid,
                 "LF: T=%dms N=%d CN0=%.2f s=%.3f a=%.3f j=%.3f",
                 (int)loop_params[lp_idx]->coherent_ms,
                 profile->mean_cnt,
                 profile->filt_val[3],
                 profile->filt_val[0],
                 profile->filt_val[1],
                 profile->filt_val[2]
                );

    profile->mean_acc[0] = profile->prev_val[0] * profile->prev_val[0];
    profile->mean_acc[1] = profile->prev_val[1] * profile->prev_val[1];
    profile->mean_acc[2] = profile->prev_val[2] * profile->prev_val[2];
    profile->mean_acc[3] = profile->prev_val[3] * profile->prev_val[3];
    profile->mean_cnt = 1;
  }
}

/**
 * Initializes the subsystem.
 *
 * This method shall be invoked before any other methods from the sybsystem.
 *
 * \return 0  On success.
 * \return -1 On error.
 */
tp_result_e tp_init()
{
  /** TODO refactor as needed */
  memset(profiles_gps1, 0, sizeof(profiles_gps1));

  return TP_RESULT_SUCCESS;
}

/**
 * Registers GNSS satellite in facility.
 *
 * The method registers GNSS signal and returns initial tracking parameters.
 *
 * \param[in]  sid    GNSS signal identifier.
 * \param[in]  data   Initial parameters.
 * \param[out] config Container for initial tracking parameters.
 *
 * \retval TP_RESULT_SUCCESS The satellite has been registered and initial
 *                           profile is returned.
 * \retval TP_RESULT_ERROR   On error.
 *
 * \sa tp_tracking_stop()
 */
tp_result_e tp_tracking_start(gnss_signal_t sid,
                              const tp_report_t *data,
                              tp_config_t *config)
{
  tp_result_e res = TP_RESULT_ERROR;

  if (NULL != config) {
    tp_profile_internal_t *profile = allocate_profile(sid);
    if (NULL != profile) {
      log_debug_sid(sid, "New tracking profile");

      profile->last_report = *data;
      profile->filt_val[0] = compute_speed(sid, data);
      profile->filt_val[1] = 0.;
      profile->filt_val[2] = 0.;
      profile->filt_val[3] = data->cn0;

      profile->mean_acc[0] = 0;
      profile->mean_acc[1] = 0;
      profile->mean_acc[2] = 0;
      profile->mean_acc[3] = 0;
      profile->mean_cnt = 0;

      profile->cur_profile_i = TP_PROFILE_INI;
      profile->cur_profile_d = TP_PROFILE_MED;
      profile->next_profile_i = TP_PROFILE_INI;
      profile->next_profile_d = TP_PROFILE_MED;

      float loop_freq = 1000.f / loop_params[0]->coherent_ms;
      lp1_filter_init(&profile->speed_filter, profile->filt_val[0], 0.6f, loop_freq);
      lp1_filter_init(&profile->accel_filter, profile->filt_val[1], 0.6f, loop_freq);
      lp1_filter_init(&profile->jitter_filter, profile->filt_val[2], 0.6f, loop_freq);
      lp1_filter_init(&profile->cn0_filter, profile->filt_val[3], 0.6f, loop_freq);

      get_profile_params(sid,
                         profile->cur_profile_i, profile->cur_profile_d,
                         config);
      res = TP_RESULT_SUCCESS;
    } else {
      log_error_sid(sid, "Can't allocate tracking profile");
    }
  }
  return res;
}

/**
 * Marks GNSS satellite as untracked.
 *
 * The method shall be invoked when tracking loop is terminated.
 *
 * \param[in] sid  GNSS signal identifier. This identifier must be registered
 *                 with a call to #tp_tracking_start().
 *
 * \retval TP_RESULT_SUCCESS On success.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_tracking_stop(gnss_signal_t sid)
{
  tp_result_e res = TP_RESULT_ERROR;
  log_debug_sid(sid, "Removing tracking profile");
  delete_profile(sid);
  res = TP_RESULT_SUCCESS;
  return res;
}

/**
 * Retrieves new tracking profile if available.
 *
 * \param[in]  sid    GNSS signal identifier. This identifier must be registered
 *                    with a call to #tp_tracking_start().
 * \param[out] config Container for new tracking parameters.
 *
 * \retval TP_RESULT_SUCCESS New tracking profile has been retrieved. The
 *                           tracking loop shall reconfigure it's components
 *                           and, possibly, change the operation mode.
 * \retval TP_RESULT_NO_DATA New tracking profile is not available. No further
 *                           actions are needed.
 * \retval TP_RESULT_ERROR   On error.
 */
tp_result_e tp_get_profile(gnss_signal_t sid, tp_config_t *config)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != config && NULL != profile) {
    if (profile->profile_update) {
      profile->profile_update = 0;
      profile->cur_profile_i = profile->next_profile_i;
      profile->cur_profile_d = profile->next_profile_d;
      get_profile_params(sid, profile->cur_profile_i, profile->cur_profile_d, config);
      u8 lp_idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];
      float loop_freq = 1000.f / loop_params[lp_idx]->coherent_ms;

      lp1_filter_init(&profile->speed_filter, profile->filt_val[0], 0.6f, loop_freq);
      lp1_filter_init(&profile->accel_filter, profile->filt_val[1], 0.6f, loop_freq);
      lp1_filter_init(&profile->jitter_filter, profile->filt_val[2], 0.6f, loop_freq);
      lp1_filter_init(&profile->cn0_filter, profile->filt_val[3], 0.1f, loop_freq);

      res = TP_RESULT_SUCCESS;
    } else {
      res = TP_RESULT_NO_DATA;
    }
  }
  return res;
}

/**
 * Method to check if there is a pending profile change.
 *
 * \param[in] sid GNSS satellite id.
 *
 * \retval true  New profile is available.
 * \retval false No profile change is required.
 */
bool tp_has_new_profile(gnss_signal_t sid)
{
  bool res = false;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != profile) {
    res = profile->profile_update != 0;
  }
  return res;
}

/**
 * Updates track profile data with supplied information.
 *
 * The method takes tracking loop data and merges it with previously collected
 * information from other tracking loops.
 *
 * \param[in] sid  GNSS signal identifier. This identifier must be registered
 *                 with a call to #tp_tracking_start().
 * \param[in] data Tracking loop report. This data is taken for analysis and
 *                 can be asynchronously.
 *
 * \retval TP_RESULT_SUCCESS on success.
 * \retval TP_RESULT_ERROR   on error.
 */
tp_result_e tp_report_data(gnss_signal_t sid, const tp_report_t *data)
{
  tp_result_e res = TP_RESULT_ERROR;
  tp_profile_internal_t *profile = find_profile(sid);
  if (NULL != data && NULL != profile) {
    /* For now, we support only GPS L1 tracking data, and handle all data
     * synchronously.
     */
    /** TODO refactor as needed */
    /** TODO convert frequencies to speed/acceleration */
    /** TODO add stress detector */
    /** TODO add profile selection criteria */
    /** TODO add TCXO drift support */

    profile->last_report = *data;
    profile->time += data->time_ms;

    update_stats(sid, profile, data);
    print_stats(sid, profile);

    bool must_change_profile = false;
    u8  next_profile_i = 0;
    u8  next_profile_d = 0;
    const char *reason = "";

    if (!profile->profile_update) {
      if (profile->cur_profile_i == 0 &&
          profile->last_report.bsync &&
          profile->last_report.olock) {
        /* Transition from 1ms integration into 2 to 20 ms integration */
        must_change_profile = true;
        next_profile_i = 1;
        next_profile_d = profile->cur_profile_d;
        reason = "bit sync";
        profile->high_cn0_count = 0;
        profile->low_cn0_count = 0;
        profile->lock_time_ms = TP_SNR_CHANGE_LOCK_MS;
      }
      if (!must_change_profile &&
          profile->cur_profile_i != 0 &&
          profile->lock_time_ms == 0) {
        /* When running over 1ms integration, there are four transitions
         * possible:
         * - increase integration time
         * - reduce integration time
         * - tighten loop parameters
         * - loosen loop parameters
         */
        float cn0 = profile->filt_val[3];

        if (cn0 >= TP_SNR_THRESHOLD_MAX) {
          /* SNR is high - look for relaxing profile */
          if (profile->cur_profile_i > TP_PROFILE_TIME_FIRST) {
            profile->high_cn0_count++;
            profile->low_cn0_count = 0;

            if (profile->high_cn0_count == TP_SNR_THRESHOLD_LOCK) {
              reason="Upper C/N0 threshold";
              profile->high_cn0_count = 0;
              profile->lock_time_ms = TP_SNR_CHANGE_LOCK_MS;
              must_change_profile = true;
              next_profile_i = profile->cur_profile_i - 1;
              next_profile_d = profile->cur_profile_d;
            }
          }
        } else if (cn0 < TP_SNR_THRESHOLD_MIN) {
          /* SNR is low - look for more restricting profile */
          if (profile->cur_profile_i < TP_PROFILE_TIME_COUNT - 1) {
            profile->high_cn0_count = 0;
            profile->low_cn0_count++;
            if (profile->low_cn0_count == TP_SNR_THRESHOLD_LOCK) {
              reason="Lower C/N0 threshold";
              profile->low_cn0_count = 0;
              profile->lock_time_ms = TP_SNR_CHANGE_LOCK_MS;
              must_change_profile = true;
              next_profile_i = profile->cur_profile_i + 1;
              next_profile_d = profile->cur_profile_d;
            }
          }
        }
      }
    }
    if (!must_change_profile) {
      /* Profile lock time count down */
      if (profile->lock_time_ms > data->time_ms) {
        profile->lock_time_ms -= data->time_ms;
      } else {
        profile->lock_time_ms = 0;
      }
    } else {
      /* Profile update scheduling */
      profile->profile_update = true;
      profile->next_profile_i = next_profile_i;
      profile->next_profile_d = next_profile_d;

      u8 lp1_idx = profile_matrix[profile->cur_profile_i][profile->cur_profile_d];
      u8 lp2_idx = profile_matrix[profile->next_profile_i][profile->next_profile_d];

      log_info_sid(sid,
                   "Profile change: %dms [%d][%d]->%dms [%d][%d] r=%s",
                   (int)loop_params[lp1_idx]->coherent_ms,
                   profile->cur_profile_i, profile->cur_profile_d,
                   (int)loop_params[lp2_idx]->coherent_ms,
                   profile->next_profile_i, profile->next_profile_d,
                   reason
                   );
    }
    res = TP_RESULT_SUCCESS;
  }
  return res;
}

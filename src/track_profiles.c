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

#include <string.h>

#define TP_MAX_SUPPORTED_SVS 10

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
  u32           cur_profile:4;   /**< Index of the currently active profile */
  u32           next_profile:4;  /**< Index of the next selected profile */
  // u64         profile_update_time; /**< Timestamp */
  float         filt_val[4];  /**< Filtered counters */
  float         mean_acc[4];  /**< Mean accumulators */
  u32           mean_cnt;     /**< Mean value divider */
  lp1_filter_t  speed_filter;  /**< Value filter */
  lp1_filter_t  accel_filter;  /**< Value filter */
  lp1_filter_t  jitter_filter; /**< Value filter */
  lp1_filter_t  cn0_filter;    /**< Value filter */
  u32           time;          /**< Tracking time */
  u32           last_time;     /**< Last stage switch time */
} tp_profile_internal_t;

/**
 * GPS satellite profiles.
 */
static tp_profile_internal_t profiles_gps1[TP_MAX_SUPPORTED_SVS] _CCM;

/**
 * C/N0 profile
 */
static const tp_cn0_params_t cn0_params_default = {
  .track_cn0_drop_thres = 31.f,
  .track_cn0_use_thres = 31.f
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
  .carr_bw = 40,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 5,
  .mode = TP_TM_PIPELINING
};
/**
 * 2 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_2ms = {
  // "(2 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))"

  .coherent_ms = 2,
  .carr_bw = 50,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_PIPELINING
};
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
/**
 * 20 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_10ms = {
  //  "(10 ms, (1, 0.7, 1, 1540), (30, 0.7, 1, 0))"

  .coherent_ms = 10,
  .carr_bw = 30,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};
/**
 * 20 ms tracking parameters for GPS L1 C/A
 */
static const tp_loop_params_t loop_params_20ms = {
  //  "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))"

  .coherent_ms = 20,
  .carr_bw = 12,
  .carr_zeta = .7f,
  .carr_k = 1,
  .carr_to_code = 1540,
  .code_bw = 1,
  .code_zeta = 0.7f,
  .code_k = 1,
  .carr_fll_aid_gain = 0,
  .mode = TP_TM_ONE_PLUS_N
};

static const tp_loop_params_t *loop_params[] = {
  &loop_params_initial,
  &loop_params_2ms,
  &loop_params_5ms,
  &loop_params_10ms,
  &loop_params_20ms,
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
                               u32           profile,
                               tp_config_t  *config)
{
  log_debug_sid(sid, "Activating profile %u", profile);

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

    config->cn0_params.track_cn0_drop_thres -= cn0_delta;
    config->cn0_params.track_cn0_use_thres -= cn0_delta;
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

      profile->mean_acc[0] = profile->filt_val[0] * profile->filt_val[0];
      profile->mean_acc[1] = 0;
      profile->mean_acc[2] = 0;
      profile->mean_acc[3] = profile->filt_val[3] * profile->filt_val[3];
      profile->mean_cnt = 1;

      profile->cur_profile = 0;
      profile->next_profile = 0;

      float loop_freq = 1000.f / loop_params[0]->coherent_ms;
      lp1_filter_init(&profile->speed_filter, profile->filt_val[0], 0.6f, loop_freq);
      lp1_filter_init(&profile->accel_filter, profile->filt_val[1], 0.6f, loop_freq);
      lp1_filter_init(&profile->jitter_filter, profile->filt_val[2], 0.6f, loop_freq);
      lp1_filter_init(&profile->cn0_filter, profile->filt_val[3], 0.6f, loop_freq);

      get_profile_params(sid, 0, config);
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
      profile->cur_profile = profile->next_profile;
      get_profile_params(sid, profile->cur_profile, config);

      float loop_freq = 1000.f / loop_params[profile->cur_profile]->coherent_ms;

      lp1_filter_init(&profile->speed_filter, profile->filt_val[0], 0.6f, loop_freq);
      lp1_filter_init(&profile->accel_filter, profile->filt_val[1], 0.6f, loop_freq);
      lp1_filter_init(&profile->jitter_filter, profile->filt_val[2], 0.6f, loop_freq);
      lp1_filter_init(&profile->cn0_filter, profile->filt_val[3], 0.6f, loop_freq);

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
    float dt_sec = 0.001f * loop_params[profile->cur_profile]->coherent_ms; //  (float)data->sample_count / SAMPLE_FREQ;
    float speed, accel, jitter, cn0;

    speed = compute_speed(sid, data);
    speed = lp1_filter_update(&profile->speed_filter, speed);
    accel = (profile->filt_val[0] - speed) / dt_sec;
    accel = lp1_filter_update(&profile->accel_filter, accel);
    jitter = (profile->filt_val[1] - accel) / dt_sec;
    jitter = lp1_filter_update(&profile->jitter_filter, jitter);
    cn0 = lp1_filter_update(&profile->cn0_filter, data->cn0);

    profile->filt_val[0] = speed;
    profile->filt_val[1] = accel;
    profile->filt_val[2] = jitter;
    profile->filt_val[3] = cn0;

    profile->mean_acc[0] += speed * speed;
    profile->mean_acc[1] += accel * accel;
    profile->mean_acc[2] += jitter * jitter;
    profile->mean_acc[3] += cn0 * cn0;
    profile->mean_cnt += 1;

    bool must_change_profile = false;

    if (!profile->profile_update) {
      if (profile->cur_profile == 0 &&
          profile->last_report.bsync &&
          profile->last_report.olock) {
        /* Transition from 1ms integration into 2 to 20 ms integration */
        must_change_profile = true;
      }
      if (!must_change_profile &&
          profile->cur_profile != 0) {
        /* When running over 1ms integration, there are four transitions
         * possible:
         * - increase integration time
         * - reduce integration time
         * - tighten loop parameters
         * - loosen loop parameters
         */
        if (profile->time - profile->last_time >= 10000) {
          profile->last_time = profile->time;
          must_change_profile = true;

          float div = 1.f;
          if (profile->mean_cnt > 0)
            div = 1.f / profile->mean_cnt;

          float s = sqrtf(profile->mean_acc[0] * div);
          float a = sqrtf(profile->mean_acc[1] * div);
          float j = sqrtf(profile->mean_acc[2] * div);
          float c = sqrtf(profile->mean_acc[3] * div);

          log_info_sid(sid,
                       "MV: T=%dms N=%d CN0=%.2f s=%.3f a=%.3f j=%.3f",
                       (int)loop_params[profile->cur_profile]->coherent_ms,
                       profile->mean_cnt,
                       c, s, a, j
                      );

          profile->mean_acc[0] = profile->filt_val[0] * profile->filt_val[0];
          profile->mean_acc[1] = profile->filt_val[1] * profile->filt_val[1];
          profile->mean_acc[2] = profile->filt_val[2] * profile->filt_val[2];
          profile->mean_acc[3] = profile->filt_val[3] * profile->filt_val[3];
          profile->mean_cnt = 1;
        }
      }
    }
    if (must_change_profile) {

      profile->profile_update = true;
      profile->next_profile = profile->cur_profile + 1;

      if (profile->next_profile > 4)
        profile->next_profile = 1;

      log_info_sid(sid,
                   "Profile change, s=%.3f a=%.3f j=%.3f ms=%d",
                   speed, accel, jitter,
                   (int)loop_params[profile->next_profile]->coherent_ms);
    }
    res = TP_RESULT_SUCCESS;
  }
  return res;
}

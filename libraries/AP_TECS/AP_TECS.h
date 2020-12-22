/// @file    AP_TECS.h
/// @brief   Combined Total Energy Speed & Height Control. This is a instance of an
/// AP_SpdHgtControl class

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Landing/AP_Landing.h>
#include <Filter/LowPassFilter2p.h>

class AP_TECS : public AP_SpdHgtControl {
public:
    AP_TECS(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, const AP_Landing &landing)
        : _ahrs(ahrs)
        , aparm(parms)
        , _landing(landing)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_TECS(const AP_TECS &other) = delete;
    AP_TECS &operator=(const AP_TECS&) = delete;

    // Update of the estimated height and height rate internal state
    // Update of the inertial speed rate internal state
    // Should be called at 50Hz or greater
    void update_50hz(void) override;

    // Update the control loop calculations
    // Do not call slower than 10Hz or faster than 500Hz
    void update_pitch_throttle(int32_t hgt_dem_cm,
                               int32_t EAS_dem_cm,
                               enum AP_Vehicle::FixedWing::FlightStage flight_stage,
                               float distance_beyond_land_wp,
                               int32_t ptchMinCO_cd,
                               int16_t throttle_nudge,
                               float hgt_afe,
                               float load_factor) override;

    // demanded throttle in percentage
    // should return -100 to 100, usually positive unless reverse thrust is enabled via _THRminf < 0
    int32_t get_throttle_demand(void) override {
        return int32_t(_throttle_dem * 100.0f);
    }

    // demanded pitch angle in centi-degrees
    // should return between -9000 to +9000
    int32_t get_pitch_demand(void) override {
        return int32_t(_pitch_dem * 5729.5781f);
    }

    // Rate of change of velocity along X body axis in m/s^2
    float get_VXdot(void) override {
        return _vel_dot_hpf_out;
    }

    // return current target airspeed
    float get_target_airspeed(void) const override {
        return _TAS_dem_adj / _ahrs.get_EAS2TAS();
    }

    // return maximum climb rate
    float get_max_climbrate(void) const override {
        return _maxClimbRate;
    }

    // return maximum sink rate (+ve number down)
    float get_max_sinkrate(void) const override {
        return _maxSinkRate;
    }
    
    // added to let SoaringContoller reset pitch integrator to zero
    void reset_pitch_I(void) override {
        _integSEB_state = 0.0f;
    }
    
    // return landing sink rate
    float get_land_sinkrate(void) const override {
        return _land_sink;
    }

    // return landing airspeed
    float get_land_airspeed(void) const override {
        return _landAirspeed;
    }

    // return height rate demand, in m/s
    float get_height_rate_demand(void) const {
        return _hgt_rate_dem;
    }

    // set path_proportion
    void set_path_proportion(float path_proportion) override {
        _path_proportion = constrain_float(path_proportion, 0.0f, 1.0f);
    }

    // set soaring flag
    void set_gliding_requested_flag(bool gliding_requested) override {
        _flags.gliding_requested = gliding_requested;
    }

    // set propulsion failed flag
    void set_propulsion_failed_flag(bool propulsion_failed) override {
        _flags.propulsion_failed = propulsion_failed;
    }


    // set pitch max limit in degrees
    void set_pitch_max_limit(int8_t pitch_limit) {
        _pitch_max_limit = pitch_limit;
    }

    // force use of synthetic airspeed for one loop
    void use_synthetic_airspeed(void) {
        _use_synthetic_airspeed_once = true;
    }

    // reset on next loop
    void reset(void) override {
        _need_reset = true;
    }

    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;

    // Last time update_speed was called
    uint64_t _update_speed_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    const AP_Vehicle::FixedWing &aparm;

    // reference to const AP_Landing to access it's params
    const AP_Landing &_landing;
    
    // TECS tuning parameters
    AP_Float _hgtCompFiltOmega;
    AP_Float _spdCompFiltOmega;
    AP_Float _maxClimbRate;
    AP_Float _minSinkRate;
    AP_Float _maxSinkRate;
    AP_Float _timeConst;
    AP_Float _landTimeConst;
    AP_Float _ptchDamp;
    AP_Float _land_pitch_damp;
    AP_Float _landDamp;
    AP_Float _thrDamp;
    AP_Float _land_throttle_damp;
    AP_Float _integGain;
    AP_Float _integGain_takeoff;
    AP_Float _integGain_land;
    AP_Float _vertAccLim;
    AP_Float _rollComp;
    AP_Float _spdWeight;
    AP_Float _spdWeightLand;
    AP_Float _landThrottle;
    AP_Float _landAirspeed;
    AP_Float _land_sink;
    AP_Float _land_sink_rate_change;
    AP_Int8  _pitch_max;
    AP_Int8  _pitch_min;
    AP_Int8  _land_pitch_max;
    AP_Float _maxSinkRate_approach;
    AP_Int32 _options;
    AP_Int8  _land_pitch_trim;
    AP_Float _flare_holdoff_hgt;
    AP_Float _hgt_dem_tconst;
    AP_Float _trim_aoa;

    enum {
        OPTION_GLIDER_ONLY=(1<<0),
    };

    AP_Float _pitch_ff_v0;
    AP_Float _pitch_ff_k;

    // temporary _pitch_max_limit. Cleared on each loop. Clear when >= 90
    int8_t _pitch_max_limit = 90;
    
    // current height estimate (above field elevation)
    float _height;

    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;

    // pitch angle demand in radians
    float _pitch_dem;

    // estimated climb rate (m/s)
    float _climb_rate;

    /*
      a filter to estimate climb rate if we don't have it from the EKF
     */
    struct {
        // height filter second derivative
        float dd_height;

        // height integration
        float height;
    } _height_filter;

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;

    // Integrator state 6 - throttle integrator
    float _integTHR_state;

    // Integrator state 6 - pitch integrator
    float _integSEB_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Filter states for rate of change of speed along X axis
    float _vel_dot_lpf_out; // output from low pass filter
    float _vel_dot_hpf_in;  // previous input to high pass filter
    float _vel_dot_hpf_out; // output from high pass filter

    // Equivalent airspeed
    float _EAS;

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current true airspeed demand after limiting
    float _TAS_dem;

    // Current true airspeed demand after low pass filtering
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_lpf;

    // LPF applied to demanded airspeed after it has been rate limited
    LowPassFilter2pFloat _TAS_dem_filter;

    // Equivalent airspeed demand
    float _EAS_dem;

    // Conversion from EAS to TAS
    float _EAS2TAS;

    // height demands
    float _hgt_dem_in;          // height demand input from autopilot (m)
    float _hgt_dem_in_prev;     // previous value of _hgt_dem_in (m)
    float _hgt_dem_lpf;         // height demand after application of low pass filtering (m)
    float _flare_hgt_dem_adj;   // height rate demand duirng flare adjusted for height tracking offset at flare entry (m)
    float _flare_hgt_dem_ideal; // height we want to fly at during flare (m)
    float _hgt_dem;             // height demand sent to control loops (m)

    // height rate demands
    float _hgt_dem_rate_ltd;    // height demand after application of the rate limiter (m)
    float _hgt_rate_dem;        // height rate demand sent to control loops

    // offset applied to height demand post takeoff to compensate for height demand filter lag
    float _post_TO_hgt_offset;

    // last lag compensation offset applied to height demand
    float _lag_comp_hgt_offset;

    // Speed demand after application of rate limiting
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;

    // Total energy rate filter state
    float _STEdotErrLast;

    struct flags {
        // Underspeed condition
        bool underspeed:1;

        // Bad descent condition caused by unachievable airspeed demand
        bool badDescent:1;

        // true when plane is in auto mode and executing a land mission item
        bool is_doing_auto_land:1;

        // true when we have reached target speed in takeoff
        bool reached_speed_takeoff:1;

        // true if the soaring feature has requested gliding flight
        bool gliding_requested:1;

        // true when we are in gliding flight, in one of three situations;
        //   - THR_MAX=0
        //   - gliding has been requested e.g. by soaring feature
        //   - engine failure detected (detection not implemented currently)
        bool is_gliding:1;

        // true if a propulsion failure is detected.
        bool propulsion_failed:1;
    };
    union {
        struct flags _flags;
        uint8_t _flags_byte;
    };

    // time when underspeed started
    uint32_t _underspeed_start_ms;

    // auto mode flightstage
    enum AP_Vehicle::FixedWing::FlightStage _flight_stage;

    // pitch demand before limiting
    float _pitch_dem_unc;

    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;
    float _STEdot_min;

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;

    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;
    float _PITCHminf;

    // Specific energy quantities
    float _SPE_dem;
    float _SKE_dem;
    float _SPEdot_dem;
    float _SKEdot_dem;
    float _SPE_est;
    float _SKE_est;
    float _SPEdot;
    float _SKEdot;

    // misc variables used for alternative precision landing pitch control
    float _hgt_rate_err_integ;
    float _hgt_at_start_of_flare;
    float _hgt_rate_at_flare_entry;
    float _hgt_afe;
    float _pitch_min_at_flare_entry;

    // Specific energy error quantities
    float _STE_error;

    // Time since last update of main TECS loop (seconds)
    float _DT;

    // counter for demanded sink rate on land final
    uint8_t _flare_counter;

    // slew height demand lag filter value when transition to land
    float hgt_dem_lag_filter_slew;

    // percent traveled along the previous and next waypoints
    float _path_proportion;

    float _distance_beyond_land_wp;

    float _land_pitch_min = -90;

    // need to reset on next loop
    bool _need_reset;

    float _SKE_weighting;

    // internal variables to be logged
    struct {
        float SPE_error;
        float SKE_error;
        float SEB_delta;
    } logging;

    AP_Int8 _use_synthetic_airspeed;

    enum class Saturation {
        NONE = 0,
        LOW = 1,
        HIGH = 2
    };

    Saturation _pitch_rate_clip_state;
    
    // use synthetic airspeed for next loop
    bool _use_synthetic_airspeed_once;
    
    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float load_factor);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Update the demanded height
    void _update_height_demand(void);

    // Detect an underspeed condition
    void _detect_underspeed(void);

    // Update Specific Energy Quantities
    void _update_energies(void);

    // Update Demanded Throttle
    void _update_throttle_with_airspeed(void);

    // Update Demanded Throttle Non-Airspeed
    void _update_throttle_without_airspeed(int16_t throttle_nudge);

    // get integral gain which is flight_stage dependent
    float _get_i_gain(void);

    // Detect Bad Descent
    void _detect_bad_descent(void);

    // Update Demanded Pitch Angle
    void _update_pitch(void);

    // Initialise states and variables
    void _initialise_states(int32_t ptchMinCO_cd, float hgt_afe);

    // Calculate specific total energy rate limits
    void _update_STE_rate_lim(void);

    // current time constant
    float timeConstant(void) const;
};

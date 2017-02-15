// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_Gps_hpp
#define msr_air_copter_sim_Gps_hpp

#include <random>
#include "common/Common.hpp"
#include "GpsSimpleParams.hpp"
#include "GpsBase.hpp"
#include "common/FirstOrderFilter.hpp"
#include "common/FrequencyLimiter.hpp"
#include "common/DelayLine.hpp"


namespace msr { namespace airlib {

class GpsSimple : public GpsBase {
public: //methods
    GpsSimple()
    {
        GpsSimple::reset();
    }
    GpsSimple(GroundTruth* ground_truth)
    {
        initialize(ground_truth);
    }
    void initialize(GroundTruth* ground_truth)
    {
        GpsBase::initialize(ground_truth);

        //initialize frequency limiter
        freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
        delay_line_.initialize(params_.update_latency);
        
        //initialize filters
        eph_filter.initialize(params_.eph_time_constant, params_.eph_final, params_.eph_initial); //starting dilution set to 100 which we will reduce over time to targetted 0.3f, with 45% accuracy within 100 updates, each update occuring at 0.2s interval
        epv_filter.initialize(params_.epv_time_constant, params_.epv_final, params_.epv_initial);
    
        GpsSimple::reset();
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        freq_limiter_.reset();
        delay_line_.reset();

        eph_filter.reset();
        epv_filter.reset();

        addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput(), 0);
    }

    virtual void update(real_T dt) override
    {
        freq_limiter_.update(dt);
        eph_filter.update(dt);
        epv_filter.update(dt);

        if (freq_limiter_.isWaitComplete()) {   //update output
            addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput(), freq_limiter_.getLastElapsedIntervalSec());
        }

        delay_line_.update(dt);

        if (freq_limiter_.isWaitComplete())
            setOutput(delay_line_.getOutput());
    }

    //*** End: UpdatableState implementation ***//

    virtual ~GpsSimple() = default;
private:
    void addOutputToDelayLine(real_T eph, real_T epv, real_T dt)
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        //GNSS
        output.gnss.time_utc = static_cast<long>(freq_limiter_.getTimestamp());
        output.gnss.geo_point = ground_truth.environment->getState().geo_point;
        output.gnss.eph = eph;
        output.gnss.epv = epv;
        output.gnss.velocity = ground_truth.kinematics->twist.linear;
        output.is_valid = true;

        output.gnss.fix_type =
            output.gnss.eph <= params_.eph_min_3d ? GnssFixType::GNSS_FIX_3D_FIX
            : output.gnss.eph <= params_.eph_min_2d ? GnssFixType::GNSS_FIX_2D_FIX
            : GnssFixType::GNSS_FIX_NO_FIX;

        delay_line_.push_back(output);
    }


private:
    typedef std::normal_distribution<> NormalDistribution;

    GpsSimpleParams params_;

    FirstOrderFilter<real_T> eph_filter, epv_filter;
    FrequencyLimiter freq_limiter_;
    DelayLine<Output> delay_line_;
};

}} //namespace
#endif

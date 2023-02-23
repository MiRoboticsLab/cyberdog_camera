// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "concurrency.h"
#include <deque>

namespace cyberdog
{
namespace camera
{
    static const double TIMESTAMP_USEC_TO_MSEC = 0.001;
    class global_time_option
    {
    public:
        global_time_option() {}
        // TODO: expose this outwards
        const char* get_description()  { return "Enable/Disable global timestamp"; }
    };

    class CSample
    {
    public:
        CSample(double x, double y) :
            _x(x), _y(y) {};
        CSample& operator-=(const CSample& other);
        CSample& operator+=(const CSample& other);

    public:
        double _x;
        double _y;
    };

    class CLinearCoefficients
    {
    public:
        CLinearCoefficients(unsigned int buffer_size);
        void reset();
        void add_value(CSample val);
        void add_const_y_coefs(double dy);
        bool update_samples_base(double x);
        void update_last_sample_time(double x);
        double calc_value(double x) const;
        bool is_full() const;

    private:
        void calc_linear_coefs();
        void get_a_b(double x, double& a, double& b) const;

    private:
        CSample _base_sample;
        unsigned int _buffer_size;
        std::deque<CSample> _last_values;
        double _prev_a, _prev_b;    //Linear regression coeffitions - previously used values.
        double _dest_a, _dest_b;    //Linear regression coeffitions - recently calculated.
        double _prev_time, _time_span_ms;
        double _last_request_time;
    };

    class global_time_interface;

    class time_diff_keeper
    {
    public:
        explicit time_diff_keeper(global_time_interface* dev, const unsigned int sampling_interval_ms);
        void start();   // must be called AFTER ALL initializations of _hw_monitor.
        void stop();
        ~time_diff_keeper();
        double get_system_hw_time(double crnt_hw_time, bool& is_ready);

    private:
        bool update_diff_time();
        void polling(dispatcher::cancellable_timer cancellable_timer);

    private:
        global_time_interface* _device;
        unsigned int _poll_intervals_ms;
        CLinearCoefficients _coefs;
        int             _users_count;
        mutable std::recursive_mutex _read_mtx; // Watch only 1 reader at a time.
        mutable std::recursive_mutex _enable_mtx; // Watch only 1 start/stop operation at a time.
        bool _is_ready;
        double _min_command_delay;
        active_object<> _active_object;
    };

    class global_timestamp_reader
    {
    public:
        global_timestamp_reader( 
                                std::shared_ptr<time_diff_keeper> timediff,
                                std::shared_ptr<global_time_option>);

        double get_frame_timestamp(double frametime) ;
        void reset() ;

    private:
        std::weak_ptr<time_diff_keeper> _time_diff_keeper;
        mutable std::recursive_mutex _mtx;
        std::shared_ptr<global_time_option> _option_is_enabled;
        bool _ts_is_ready;
    };

    class global_time_interface
    {
    public:
	std::shared_ptr<time_diff_keeper> _tf_keeper;
        global_time_interface();
        ~global_time_interface() { _tf_keeper.reset(); }
        void enable_time_diff_keeper(bool is_enable);
    };

}
}

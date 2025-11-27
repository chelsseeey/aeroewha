#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/battery_status.h>

using namespace time_literals;

class FailSafeHover : public ModuleBase<FailSafeHover>
{
public:
    FailSafeHover()
        : _sub_lpos(ORB_ID(vehicle_local_position)),
          _sub_gpos(ORB_ID(vehicle_global_position)),
          _sub_gps(ORB_ID(vehicle_gps_position)),
          _sub_status(ORB_ID(vehicle_status)),
          _sub_batt(ORB_ID(battery_status)),
          _pub_sp(ORB_ID(vehicle_rates_setpoint))
    {}

    void run() override;

    static int task_spawn(int argc, char *argv[])
    {
        FailSafeHover *instance = new FailSafeHover();
        if (!instance) return PX4_ERROR;

        _object.store(instance);

        px4_task_spawn_cmd("failsafe_hover",
                           SCHED_DEFAULT,
                           SCHED_PRIORITY_DEFAULT,
                           2200,
                           (px4_main_t)&FailSafeHover::task_main_trampoline,
                           nullptr);
        return PX4_OK;
    }

    static FailSafeHover *instantiate(int, char *[]) { return new FailSafeHover(); }
    static int custom_command(int, char *[]) { return print_usage("unknown cmd"); }

    static int print_usage(const char *) {
        PRINT_MODULE_USAGE_NAME("failsafe_hover", "module");
        PRINT_MODULE_USAGE_COMMAND("start");
        return 0;
    }

private:

    // Subscriptions
    uORB::Subscription _sub_lpos;
    uORB::Subscription _sub_gpos;
    uORB::Subscription _sub_gps;
    uORB::Subscription _sub_status;
    uORB::Subscription _sub_batt;

    // Publication
    uORB::Publication<vehicle_rates_setpoint_s> _pub_sp;

    bool _failsafe_active{false};
    float _target_alt{0.0f};       // absolute z position
    float _failsafe_hover_z{-10.f}; // default hover altitude (–10m = 약 10m 위)
};

// -------------------------------------------------------------

void FailSafeHover::run()
{
    PX4_INFO("Failsafe Hover module started");

    vehicle_local_position_s lpos{};
    vehicle_status_s status{};
    battery_status_s batt{};
    vehicle_gps_position_s gps{};
    vehicle_global_position_s gpos{};

    while (!should_exit()) {

        _sub_status.update(&status);
        _sub_batt.update(&batt);
        _sub_gps.update(&gps);
        _sub_gpos.update(&gpos);

        bool trigger = false;

        // 1) 통신 두절(RC link lost)
        if (status.rc_signal_lost) {
            PX4_WARN("Failsafe: RC signal lost!");
            trigger = true;
        }

        // 2) GPS 오류
        if (gps.fix_type < 3 || gps.eph > 2.5f) {
            PX4_WARN("Failsafe: GPS error!");
            trigger = true;
        }

        // 3) 배터리 부족
        if (batt.voltage_filtered_v < 10.5f) {
            PX4_WARN("Failsafe: Low battery!");
            trigger = true;
        }

        // 4) 금지 구역 진입 (Radius 100m 예시)
        const float nofly_lat = 37.123456;   // 금지구역 중심
        const float nofly_lon = 127.123456;
        const float radius_m = 100.0f;

        float dlat = (float)(gps.lat * 1e-7 - nofly_lat) * 111320.0f;
        float dlon = (float)(gps.lon * 1e-7 - nofly_lon) * 111320.0f * cosf(nofly_lat * M_PI / 180.0f);

        float dist = sqrtf(dlat * dlat + dlon * dlon);

        if (dist < radius_m) {
            PX4_WARN("Failsafe: Entered no-fly zone!");
            trigger = true;
        }

        // -------------------------
        // Failsafe Mode Enable
        // -------------------------
        if (trigger && !_failsafe_active) {

            PX4_WARN("Failsafe HOVER engaged!");

            if (_sub_lpos.copy(&lpos) == PX4_OK) {
                // 현재 z 기준으로 특정 절대고도 설정
                _target_alt = _failsafe_hover_z;
            }

            _failsafe_active = true;
        }

        // -------------------------
        // Failsafe Hover Control
        // -------------------------
        if (_failsafe_active) {

            if (_sub_lpos.copy(&lpos) == PX4_OK) {

                float error = (_target_alt - lpos.z);

                // 단순 P 제어 + 호버
                float thrust = error * 0.45f + 0.55f;

                if (thrust < 0.3f) thrust = 0.3f;
                if (thrust > 0.9f) thrust = 0.9f;

                vehicle_rates_setpoint_s sp{};
                sp.timestamp = hrt_absolute_time();

                sp.roll  = 0.f;
                sp.pitch = 0.f;
                sp.yaw   = 0.f;
                sp.thrust_body[2] = -thrust; // 위쪽 힘은 음수

                _pub_sp.publish(sp);
            }
        }

        px4_usleep(10_ms);
    }

    PX4_INFO("Failsafe Hover module exiting");
}

// -------------------------------------------------------------

extern "C" __EXPORT int failsafe_hover_main(int argc, char *argv[])
{
    return FailSafeHover::main(argc, argv);
}

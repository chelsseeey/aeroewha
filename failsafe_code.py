#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_command.h>

#include <math.h>

class CustomFailsafe : public ModuleBase<CustomFailsafe>
{
public:
    CustomFailsafe() = default;
    ~CustomFailsafe() = default;

    static int task_spawn(int argc, char *argv[]);
    static CustomFailsafe *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;
    void stop() { _running = false; }

private:

    bool _running{true};

    // 비행 금지 구역 설정
    const float nofly_lat = 37.123456;
    const float nofly_lon = 127.123456;
    const float nofly_radius = 100.0f;

    const float failsafe_hover_alt = 5.0f;   // 5m 위로 상승

    // 구독
    uORB::Subscription _sub_gpos{ORB_ID(vehicle_global_position)};
    uORB::Subscription _sub_lpos{ORB_ID(vehicle_local_position)};
    uORB::Subscription _sub_status{ORB_ID(vehicle_status)};
    uORB::Subscription _sub_telem{ORB_ID(telemetry_status)};
    uORB::Subscription _sub_batt{ORB_ID(battery_status)};

    // 명령 발행
    uORB::Publication<vehicle_command_s> _pub_cmd{ORB_ID(vehicle_command)};

    // 편의 함수들
    float calc_distance_m(float lat1, float lon1, float lat2, float lon2);
    void send_hover_command(float rel_alt);
    void send_RTL();
    void send_land();
};

float CustomFailsafe::calc_distance_m(float lat1, float lon1, float lat2, float lon2)
{
    constexpr float R = 6371000.0f;

    float dlat = (lat2 - lat1) * M_PI / 180.0f;
    float dlon = (lon2 - lon1) * M_PI / 180.0f;

    float a = sinf(dlat/2)*sinf(dlat/2) +
              cosf(lat1*M_PI/180.0f)*cosf(lat2*M_PI/180.0f) *
              sinf(dlon/2)*sinf(dlon/2);

    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    return R * c;
}

void CustomFailsafe::send_hover_command(float rel_alt)
{
    vehicle_command_s cmd{};
    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LOITER;
    cmd.param7 = rel_alt;
    _pub_cmd.publish(cmd);
    PX4_WARN("Failsafe: Hover at %.2f m", (double)rel_alt);
}

void CustomFailsafe::send_RTL()
{
    vehicle_command_s cmd{};
    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
    _pub_cmd.publish(cmd);
    PX4_WARN("Failsafe: RTL triggered!");
}

void CustomFailsafe::send_land()
{
    vehicle_command_s cmd{};
    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
    _pub_cmd.publish(cmd);
    PX4_WARN("Failsafe: Critical battery → Landing now!");
}

int CustomFailsafe::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("custom_failsafe",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  3000,
                                  (px4_main_t)&run_trampoline,
                                  nullptr);
    return _task_id >= 0 ? 0 : -1;
}

CustomFailsafe *CustomFailsafe::instantiate(int argc, char *argv[])
{
    return new CustomFailsafe();
}

int CustomFailsafe::custom_command(int argc, char *argv[]) { return print_usage(); }
int CustomFailsafe::print_usage(const char *reason) { PX4_INFO("Usage: custom_failsafe start"); return 0; }


void CustomFailsafe::run()
{
    PX4_INFO("Custom Failsafe module running...");

    while (_running) {

        // 1) 위치 정보
        vehicle_global_position_s gpos{};
        vehicle_local_position_s lpos{};
        vehicle_status_s status{};
        telemetry_status_s telem{};
        battery_status_s batt{};

        _sub_gpos.update(&gpos);
        _sub_lpos.update(&lpos);
        _sub_status.update(&status);
        _sub_telem.update(&telem);
        _sub_batt.update(&batt);

        // --------------------------
        // A) GPS 오류 (Position Loss)
        // --------------------------
        if (!gpos.valid || gpos.eph > 3.0f || gpos.epv > 3.0f) {
            PX4_WARN("Failsafe: GPS Lost → Hovering");
            send_hover_command( failsafe_hover_alt );
            px4_usleep(200000);
            continue;
        }

        // --------------------------
        // B) 통신 두절 (RC/Telemetry Loss)
        // --------------------------
        if (status.rc_signal_lost || telem.type == telemetry_status_s::LINK_TYPE_UNKNOWN) {
            PX4_WARN("Failsafe: RC/Telemetry Lost → RTL");
            send_RTL();
            px4_usleep(200000);
            continue;
        }

        // --------------------------
        // C) 배터리 부족
        // --------------------------
        if (batt.warning == battery_status_s::BATTERY_WARNING_LOW) {
            PX4_WARN("Battery Warning: Consider RTL soon");
        }

        if (batt.warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
            send_land();
            px4_usleep(200000);
            continue;
        }

        // --------------------------
        // D) 금지구역 진입 감지
        // --------------------------
        float dist = calc_distance_m(gpos.lat, gpos.lon, nofly_lat, nofly_lon);

        if (dist < nofly_radius) {
            PX4_WARN("Entered NO-FLY ZONE → Hover fail-safe");
            send_hover_command(failsafe_hover_alt);
            px4_usleep(200000);
            continue;
        }

        px4_usleep(20000);
    }
}


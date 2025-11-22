// file: motor_gain_pre_mixer.cpp

#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include <lib/module_params/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>


class MotorGainPreMixer : public ModuleBase<MotorGainPreMixer>, public ModuleParams
{
public:
    MotorGainPreMixer() :
        ModuleParams(nullptr),
        _controls_sub(ORB_ID(actuator_controls_0)),
        _controls_pub(ORB_ID(actuator_controls_0))
    {}

    ~MotorGainPreMixer() override {}

    static int task_spawn(int argc, char *argv[])
    {
        MotorGainPreMixer *instance = new MotorGainPreMixer();

        if (!instance) return PX4_ERROR;

        _object.store(instance);
        _task_id = px4_task_spawn_cmd(
            "motor_gain_pre_mixer",
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT,
            2000,
            (px4_main_t)&run_trampoline,
            nullptr
        );

        if (_task_id < 0) {
            delete instance;
            return PX4_ERROR;
        }

        return PX4_OK;
    }

    static MotorGainPreMixer *instantiate(int argc, char *argv[]) { return new MotorGainPreMixer(); }

    void run() override
    {
        PX4_INFO("MotorGainPreMixer started.");

        actuator_controls_s controls_in{};
        actuator_controls_s controls_out{};

        while (!should_exit()) {

            if (_controls_sub.update(&controls_in)) {

                // Copy timestamp & default values
                controls_out = controls_in;

                // Motor mapping:
                // control[0] = roll
                // control[1] = pitch
                // control[2] = yaw
                // control[3] = collective thrust (0~1 normalized)

                // Mixer combines these into motor outputs,
                // but we want to scale per motor BEFORE mixer.
                //
                // In PX4, individual motors are not separated here.
                // So we modify the thrust component ONLY.

                float thrust = controls_in.control[3];

                // Apply gain per motor by approximating desired thrust correction:
                // mixer will generate 4 motor commands from roll/pitch/yaw/thrust
                // We modify thrust based on average gain or adjust control set.

                float g1 = _g1.get();
                float g2 = _g2.get();
                float g3 = _g3.get();
                float g4 = _g4.get();

                // 하나의 thrust 값으로 4개 모터의 상대 추력을 보정하려면
                // 평균 gain 적용이 가장 논리적임.
                float g_avg = (g1 + g2 + g3 + g4) * 0.25f;

                controls_out.control[3] = thrust * g_avg;

                controls_out.timestamp = hrt_absolute_time();

                _controls_pub.publish(controls_out);
            }

            px4_usleep(2000); // ~500 Hz loop
        }

        PX4_INFO("MotorGainPreMixer stopped.");
    }

    static int custom_command(int argc, char *argv[]) { return print_usage("Unknown command"); }
    static int print_usage(const char *reason)
    {
        if (reason) PX4_WARN("%s", reason);

        PRINT_MODULE_USAGE_NAME("motor_gain_pre_mixer", "custom");
        PRINT_MODULE_USAGE_COMMAND("start");
        return 0;
    }


private:

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::MGC_G1>) _g1,
        (ParamFloat<px4::params::MGC_G2>) _g2,
        (ParamFloat<px4::params::MGC_G3>) _g3,
        (ParamFloat<px4::params::MGC_G4>) _g4
    );

    uORB::Subscription _controls_sub;
    uORB::Publication<actuator_controls_s> _controls_pub;
};


MotorGainPreMixer *MotorGainPreMixer::_object = nullptr;

extern "C" __EXPORT int motor_gain_pre_mixer_main(int argc, char *argv[])
{
    return MotorGainPreMixer::main(argc, argv);
}

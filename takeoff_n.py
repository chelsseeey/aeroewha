#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include <lib/module_params/module_params.h>
#include <parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_outputs.h>

using namespace time_literals;

class MotorGainPostMixer : public ModuleBase<MotorGainPostMixer>, public ModuleParams
{
public:
    MotorGainPostMixer() :
        ModuleParams(nullptr),
        _sub_motors(ORB_ID(actuator_motors)),
        _pub_outputs(ORB_ID(actuator_outputs))
    {
    }

    ~MotorGainPostMixer() override {}

    static int task_spawn(int argc, char *argv[])
    {
        MotorGainPostMixer *instance = new MotorGainPostMixer();
        if (!instance) {
            PX4_ERR("alloc failed");
            return PX4_ERROR;
        }

        _object.store(instance);

        _task_id = px4_task_spawn_cmd("motor_gain_post_mixer",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_DEFAULT,
                                      1700,
                                      (px4_main_t)&MotorGainPostMixer::task_trampoline,
                                      nullptr);

        if (_task_id < 0) {
            PX4_ERR("task spawn failed");
            delete instance;
            return PX4_ERROR;
        }

        return PX4_OK;
    }

    static MotorGainPostMixer *instantiate(int argc, char *argv[]) { return new MotorGainPostMixer(); }
    static int custom_command(int argc, char *argv[]) { return print_usage("Unknown command"); }
    static int print_usage(const char *reason);

    void run() override;

private:

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::MGC_EN>) _enabled,
        (ParamFloat<px4::params::MGC_G1>) _g1,
        (ParamFloat<px4::params::MGC_G2>) _g2,
        (ParamFloat<px4::params::MGC_G3>) _g3,
        (ParamFloat<px4::params::MGC_G4>) _g4
    );

    uORB::Subscription _sub_motors;
    uORB::Publication<actuator_outputs_s> _pub_outputs;

    static MotorGainPostMixer *_object;
    static int _task_id;
};

MotorGainPostMixer *MotorGainPostMixer::_object = nullptr;
int MotorGainPostMixer::_task_id = -1;

void MotorGainPostMixer::run()
{
    PX4_INFO("motor_gain_post_mixer started");

    actuator_motors_s motors{};
    actuator_outputs_s outputs{};

    // safety: initialize outputs to zero
    outputs.noutputs = 0;
    for (unsigned i = 0; i < sizeof(outputs.output)/sizeof(outputs.output[0]); i++) {
        outputs.output[i] = 0.0f;
    }

    // loop rate ~ 250-400 Hz depending on source; we'll poll with timeout
    const int wait_ms = 4; // ~250 Hz

    while (!should_exit()) {

        // if module disabled, just sleep and continue
        if (!_enabled.get()) {
            px4_usleep(wait_ms * 1000);
            // optionally we could still swallow the topic to keep it fresh
            (void)_sub_motors.update(&motors);
            continue;
        }

        // wait for actuator_motors update
        if (_sub_motors.updated()) {
            if (_sub_motors.copy(&motors) != PX4_OK) {
                px4_usleep(wait_ms * 1000);
                continue;
            }

            // Prepare outputs structure based on motors
            outputs.timestamp = hrt_absolute_time();

            // motors.noutputs indicates number of motor channels produced by mixer
            unsigned n = motors.noutputs;
            if (n > sizeof(outputs.output)/sizeof(outputs.output[0])) {
                // clamp for safety
                n = sizeof(outputs.output)/sizeof(outputs.output[0]);
            }

            outputs.noutputs = n;

            // Gains from params (default 1.0)
            float gains[4] = { _g1.get(), _g2.get(), _g3.get(), _g4.get() };

            // Apply per-motor gains for available channels.
            // motors.control[] holds the post-mixer normalized outputs (0..1 or -1..1 depending on reversible flags).
            for (unsigned i = 0; i < n; ++i) {
                float base = motors.control[i];

                // if we only specified 4 gains but more channels exist, apply 1.0 for extra channels
                float g = (i < 4) ? gains[i] : 1.0f;

                // apply gain
                float scaled = base * g;

                // safety clamp to reasonable range: [-1.0, 1.0] (actuator_outputs expects normalized outputs)
                if (scaled > 1.0f) scaled = 1.0f;
                if (scaled < -1.0f) scaled = -1.0f;

                outputs.output[i] = scaled;
            }

            // publish actuator_outputs (this will be used by IO/PWM driver)
            if (!_pub_outputs.advertised()) {
                _pub_outputs.advertise(outputs);
            } else {
                _pub_outputs.publish(outputs);
            }
        }

        px4_usleep(wait_ms * 1000);
    }

    PX4_INFO("motor_gain_post_mixer exiting");
}

int MotorGainPostMixer::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_USAGE_NAME("motor_gain_post_mixer", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('h', 0, 0, "unused", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int motor_gain_post_mixer_main(int argc, char *argv[])
{
    return MotorGainPostMixer::main(argc, argv);
}

// Copyright (c) 2017, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <vehicle_manager.h>

#include <ugcs/vsm/run_as_service.h>

#ifdef __unix__
#include <signal.h>
#endif /* __unix__ */

using namespace ugcs::vsm;

Vehicle_manager vm;
/** Separate main thread when running as service. */
std::thread main_thread;

#ifdef __unix__
void Sigint_handler(int signum __UNUSED)
{
    LOG_INFO("Signal caught, exiting...");
    vm.Stop();
}
#endif /* __unix__ */

void
Main(int argc, char **argv)
{
    Initialize(argc, argv, "vsm-mikrokopter.conf");
    vm.Enable();
    vm.Run();
    vm.Disable();
    Terminate();
}

int
Start_main(int argc, char *argv[])
{
    main_thread = std::thread(Main, argc, argv);
    return 0;
}

void
Stop_main()
{
    LOG_INFO("Service stop requested, exiting...");
    vm.Stop();
    main_thread.join();
}

int
main(int argc, char *argv[])
{
    auto ret = Run_as_service("ugcs-vsm-mikrokopter", argc, argv,
                              Make_program_init_handler(Start_main),
                              Make_callback(Stop_main));
    if (ret != SERVICE_RESULT_NORMAL_INVOCATION) {
        return ret;
    }

#ifdef __unix__
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = Sigint_handler;
    sigaction(SIGINT, &action, NULL);
#endif /* __unix__ */

    Main(argc, argv);

    return 0;
}

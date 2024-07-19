#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

using namespace std::chrono_literals;

/* make some talons for drive train */
TalonSRX talRght(0);

void initDrive()
{
    /* both talons should blink green when driving forward */
    talRght.SetInverted(true);
}

int main()
{
    /* don't bother prompting, just use can0 */
    // std::cout << "Please input the name of your can interface: ";
    auto interface = "can0";
    if(ctre::phoenix::platform::can::SetCANInterface(interface) != 0)
    {
        printf("Set Interface failed!");
        std::exit(-1);
    }

    // Comment out the call if you would rather use the automatically running
    // diag-server, note this requires uninstalling diagnostics from Tuner.
    // c_SetPhoenixDiagnosticsStartTime(-1); // disable diag server, instead we will use the diag server stand alone application that Tuner installs

    while(true)
    {
        double percent = 0;
        for(size_t i = 0; i < 100; i++)
        {
            /* code */
            talRght.Set(ControlMode::PercentOutput, percent);
            percent += 1;
            std::this_thread::sleep_for(1ms);
        }

        for(size_t i = 0; i < 100; i++)
        {
            /* code */
            talRght.Set(ControlMode::PercentOutput, percent);
            percent -= 1;
            std::this_thread::sleep_for(1ms);
        }
    }

    return 0;
}
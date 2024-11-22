#include "esminiLib.hpp"
/*
int main(int argc, char* argv[])
{
    if (argc > 1)
    {
        SE_InitWithArgs(argc, const_cast<const char**>(argv));
    }
    else
    {
        SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);
    }

    for (int i = 0; i < 500; i++)
    {
        SE_Step();
    }

    SE_Close();

    return 0;
} */

//#include "./EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp"
#include <signal.h>
#include <stdio.h>

static int quit_flag = 0;

static void signal_handler(int s)
{
    if (s == SIGINT)
    {
        printf("Quit request from user\n");
        quit_flag = 1;
    }
}

int main()
{
    double elapsed_time = 0;
    bool change_options = true;
    signal(SIGINT, signal_handler);
    SE_SetLogFilePath("my_test.txt");
    SE_SetOptionPersistent("log_append");
    SE_SetOption("osi_file");
    SE_SetOptionValue("log_level", "info");
    SE_SetOptionValue("fixed_timestep", "0.01");
    SE_Init("../../../../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

    while (SE_GetQuitFlag() == 0 && !quit_flag)
    {
        if (change_options && elapsed_time >= 3.0)
        {
            printf("Changing options\n");
            SE_SetOptionPersistent("log_meta_data");
            // SE_SetOptionPersistent("osi_file");
            change_options = false;
        }
        SE_Step();
        elapsed_time += 0.01;
    }
    SE_Close();
    SE_SetLogFilePath("my_test_error.txt");
    // SE_SetOptionValuePersistent("log_level", "error");
    SE_SetOptionPersistent("osi_file"); // Works
    SE_Init("../../../../resources/xosc/cut-in.xosc", 0, 1, 0, 0);
    
    while (SE_GetQuitFlag() == 0 && !quit_flag)
    {
        SE_Step();
    }
    SE_Close();
    return 0;
}
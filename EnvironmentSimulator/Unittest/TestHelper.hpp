#pragma once

#include "CommonMini.hpp"
#include "logger.hpp"
#include "Utils.h"


#include <gtest/gtest.h>


static int ParseAndSetLoggerOptions(int argc, char** argv)
{
    SE_Options opt;

    opt.AddOption("disable_stdout", "Prevent messages to stdout", "yes/no", "yes");
    opt.AddOption("log_append", "log all scenarios in the same txt file");
    opt.AddOption("log_meta_data", "log file name, function name and line number");
    opt.AddOption("log_level", "log level debug, info, warn, error", "mode");
    opt.AddOption("log_only_modules", "log from only these modules. Overrides logSkip_Modules", "modulename(s)");
    opt.AddOption("log_skip_modules", "skip log from these modules, all remaining modules will be logged.", "modulename(s)");    

    if (opt.ParseArgs(argc, argv) != 0)
    {
        opt.PrintUsage();
        return -2;
    }

    if (opt.HasUnknownArgs())
    {
        opt.PrintUnknownArgs("Unrecognized arguments:");
        opt.PrintUsage();
        return -3;
    }

    //LoggerConfig logConfig;
    std::string  arg_str;
    if (opt.IsOptionArgumentSet("log_only_modules"))
    {
        arg_str             = opt.GetOptionArg("log_only_modules");
        const auto splitted = utils::SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            loggerConfig.enabledFiles_.insert(splitted.begin(), splitted.end());
        }
    }
    if (opt.IsOptionArgumentSet("log_Skip_Modules"))
    {
        arg_str             = opt.GetOptionArg("log_Skip_Modules");
        const auto splitted = utils::SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            loggerConfig.disabledFiles_.insert(splitted.begin(), splitted.end());
        }
    }

    //SetupLogger(logConfig);
    return 0;
}

class CommonTestSetup : public testing::EmptyTestEventListener
{

public:
    CommonTestSetup(int argc, char** argv)
    : argc_(argc), argv_(argv)
    {
        //std::cout << " CommonTestSetup called " << std::endl;        
    }
    // Called before a test starts.
    void OnTestStart( const testing::TestInfo& test_info ) override
    {   
        std::cout << " OnTestStart called " << std::endl;
        (void)test_info;
        if (ParseAndSetLoggerOptions(argc_, argv_) != 0)
        {
            exit(-1);
        }

    }

    // Called after a test ends.
    void OnTestEnd( const testing::TestInfo& test_info ) override
    {    
        (void)test_info;    
        //std::cout << " OnTestEnd called " << std::endl;
    }

// private members
private:
    
    int argc_;
    char** argv_;

};





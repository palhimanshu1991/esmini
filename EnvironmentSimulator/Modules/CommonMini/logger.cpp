/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "logger.hpp"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/fmt/fmt.h"

#include <iostream>
#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error "Missing <filesystem> header"
#endif

#include <unordered_set>

std::shared_ptr<spdlog::logger> consoleLogger;
std::shared_ptr<spdlog::logger> fileLogger;
static std::string              strTime;
static std::string              currentLogFileName;
static std::string              defaultLogFileName = "log.txt";

LoggerConfig& LoggerConfig::Inst()
{
    static LoggerConfig loggerConfig_;
    return loggerConfig_;
}

void SetLoggerLevel(std::shared_ptr<spdlog::logger>& logger)
{
    if (!logger)
    {
        return;
    }
    if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_level"))
    {
        logger->set_level(GetLogLevelFromStr(SE_Env::Inst().GetOptions().GetOptionArg("log_level")));
    }
    else
    {
        logger->set_level(spdlog::level::info);  // we keep info level as default
    }
}

std::string HandleDirectoryAndWrongPath(const std::string& path)
{
    fs::path filePath = path;
    if (filePath.has_parent_path() && !fs::exists(filePath.parent_path()))
    {
        std::cout << "Invalid log file path, parent directory does not exist : " << filePath.string() << '\n';
        exit(-1);
    }
    if (fs::is_directory(filePath))
    {
        filePath = fmt::format("{}{}", path, defaultLogFileName);
    }
    return filePath.string();
}

void CreateFileLogger(const std::string& path)
{
    try
    {
        if ((path.empty() && currentLogFileName.empty()) || path != currentLogFileName)
        {
            bool        appendFile = SE_Env::Inst().GetOptions().IsOptionArgumentSet("log_append");
            std::string filePath   = path.empty() ? LoggerConfig::Inst().logFilePath_ : path;
            filePath               = HandleDirectoryAndWrongPath(filePath);
            fileLogger             = spdlog::basic_logger_mt("file", filePath, !appendFile);
            SetLoggerLevel(fileLogger);
            fileLogger->set_pattern("%v");
            fileLogger->error(GetVersionInfoForLog());
            currentLogFileName = filePath.empty() ? LoggerConfig::Inst().logFilePath_ : filePath;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void CreateNewFileForLogging(const std::string& path)
{
    SetLoggerLevel(fileLogger);
    SetLoggerLevel(consoleLogger);
    std::string filePath = HandleDirectoryAndWrongPath(path);
    if (filePath.empty() || currentLogFileName == filePath)
    {
        return;
    }
    if (fileLogger)
    {
        spdlog::drop("file");
        fileLogger.reset();
        currentLogFileName = "";
    }
    LogFile(filePath);
}

bool LogConsole()
{
    bool shouldLog = !SE_Env::Inst().GetOptions().IsOptionArgumentSet("disable_stdout");
    if (shouldLog && !consoleLogger)
    {
        consoleLogger = spdlog::stdout_color_mt("console");
        SetLoggerLevel(consoleLogger);
        consoleLogger->set_pattern("%v");
        consoleLogger->error(GetVersionInfoForLog());
    }
    return shouldLog;
}

bool LogFile(const std::string& providedPath)
{
    bool fileLoggerDisabled = SE_Env::Inst().GetOptions().GetOptionSet("disable_log");

    if (fileLogger)
    {
        if (fileLoggerDisabled)
        {
            spdlog::drop("file");
            fileLogger.reset();
            currentLogFileName = "";
            return false;
        }
    }
    else  // no file logging currently available
    {
        if (fileLoggerDisabled)
        {
            return false;
        }
        else
        {
            if (!providedPath.empty())
            {
                CreateFileLogger(providedPath);
                return true;
            }

            std::string filePath;
            if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("logfile_path"))
            {
                filePath = SE_Env::Inst().GetOptions().GetOptionArg("logfile_path");

                if (filePath.empty())
                {
                    return false;
                }
                else
                {
                    printf("Custom logfile path: %s\n", filePath.c_str());
                    CreateFileLogger(filePath);
                }
            }
            else
            {
                CreateFileLogger("log.txt");
            }
        }
    }
    return true;
}

void StopFileLogging()
{
    spdlog::drop("file");
    fileLogger.reset();
    currentLogFileName = "";
    spdlog::shutdown();
    LoggerConfig::Inst().enabledFiles_.clear();
    LoggerConfig::Inst().disabledFiles_.clear();
}

void StopConsoleLogging()
{
    if (consoleLogger)
    {
        spdlog::drop("console");
        consoleLogger.reset();
    }
}

spdlog::level::level_enum GetLogLevelFromStr(const std::string& str)
{
    if ("debug" == str)
    {
        return spdlog::level::debug;
    }
    else if ("info" == str)
    {
        return spdlog::level::info;
    }
    else if ("warn" == str)
    {
        return spdlog::level::warn;
    }
    else if ("error" == str)
    {
        return spdlog::level::err;
    }
    // by default we set info
    return spdlog::level::info;
}

bool ShouldLogModule(char const* file)
{
    std::string fileName = fs::path(file).stem().string();
    // it may seem that checking emptiness is an overhead as find function does it optimmally
    // but checking emptiness helps to find if user has enabled any file or not, because if its empty
    // then user wants all logs i.e. no filtering
    if (!LoggerConfig::Inst().enabledFiles_.empty())
    {
        if (LoggerConfig::Inst().enabledFiles_.find(fileName) == LoggerConfig::Inst().enabledFiles_.end())
        {
            // not found in the list, which means user has not enabled this file for logging
            return false;
        }
        else
        {
            // file is present in the enabled list, we log
            return true;
        }
    }
    if (!LoggerConfig::Inst().disabledFiles_.empty())
    {
        if (LoggerConfig::Inst().disabledFiles_.find(fileName) == LoggerConfig::Inst().disabledFiles_.end())
        {
            // not found in the list, which means this file is enabled for logging
            return true;
        }
        else
        {
            // file is present in the list, which means user has categorically disabled this file from logging
            return false;
        }
    }
    // if we are here then user doesnt have any enabled/disabled files, so we log
    return true;
}

std::string AddTimeAndMetaData(char const* function, char const* file, long line, const std::string& level, const std::string& log)
{
    if (LoggerConfig::Inst().time_ != nullptr)
    {
        strTime = fmt::format("[{:.3f}]", *LoggerConfig::Inst().time_);
    }
    else
    {
        strTime = "[]";
    }
    if( LoggerConfig::Inst().metaDataEnabled_)
    {
        std::string fileName = fs::path(file).filename().string();
        std::string logWithTimeAndMeta{fmt::format("{} [{}] [{}::{}::{}] {}", strTime, level, fileName, function, line, log)};
        return logWithTimeAndMeta;
    }
    else
    {
        std::string logWithTime{fmt::format("{} [{}] {}", strTime, level, log)};
        return logWithTime;
    }
}

void SetLoggerTime(double* ptr)
{
    LoggerConfig::Inst().time_ = ptr;
}

void LogVersion()
{
    std::string esminiVersion = GetVersionInfoForLog();
    if (LogConsole())
    {
        consoleLogger->error(esminiVersion);
    }
    if (LogFile())
    {
        fileLogger->error(esminiVersion);
    }
}

void LogTimeOnly()
{
    if (LogConsole())
    {
        consoleLogger->set_pattern("[%Y-%m-%d %H:%M:%S]");
        consoleLogger->error("");
        consoleLogger->set_pattern("%v");
    }
    if (LogFile())
    {
        fileLogger->set_pattern("[%Y-%m-%d %H:%M:%S]");
        fileLogger->error("");
        fileLogger->set_pattern("%v");
    }
}
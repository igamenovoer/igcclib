#pragma once

#include "def_master.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/null_sink.h"
#include "spdlog/sinks/ostream_sink.h"
#include "spdlog/sinks/base_sink.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <map>

namespace _NS_UTILITY
{
	namespace Logging
	{
		inline constexpr const char* LOGKEY_CONSOLE() {
			return "_ns_log_console";
		}

		inline constexpr const char* LOGKEY_NULL() {
			return "_ns_log_null";
		}
		//const std::string LOGKEY_CONSOLE = "_ns_log_console";
		//const std::string LOGKEY_NULL = "_ns_log_null";

		using LoggerPtr = std::shared_ptr<spdlog::logger>;

		inline LoggerPtr get_logger(const std::string& name, const std::string& default_logkey = LOGKEY_NULL());
		inline LoggerPtr get_stock_logger(const std::string& name);
		inline LoggerPtr& get_default_logger();
		inline void set_default_logger(LoggerPtr lg);

		// ============= implementation =======================
		LoggerPtr get_logger(const std::string& name, const std::string& default_logkey)
		{
			auto p = spdlog::get(name);
			if (p)
				return p;
			else
				return get_stock_logger(default_logkey);
		}

		LoggerPtr get_stock_logger(const std::string& name)
		{
			static std::map<std::string, LoggerPtr> loggers;
			auto it = loggers.find(name);
			if (it != loggers.end()) return it->second;

			//logger not found, create one
			LoggerPtr output;
			if (name == LOGKEY_CONSOLE())
			{
				output = spdlog::stdout_color_mt(name);
				loggers[name] = output;
			}
			else if (name == LOGKEY_NULL())
			{
				output = spdlog::null_logger_mt(name);
				loggers[name] = output;
			}
			return output;
		}

		LoggerPtr& get_default_logger()
		{
			static LoggerPtr lg;
			if (lg == nullptr)
				lg = get_stock_logger(LOGKEY_CONSOLE());
			return lg;
		}

		void set_default_logger(LoggerPtr lg)
		{
			get_default_logger() = lg;
		}
	}
}
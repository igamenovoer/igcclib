#pragma once

#include <string>
#include <openssl/err.h>

namespace _NS_UTILITY {
	inline std::string get_openssl_error() {
		auto code = ERR_get_error();
		char errstr[300];
		ERR_error_string(code, errstr);
		return std::string(errstr);
	}
}
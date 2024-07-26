#pragma once

#include <igcclib/igcclib_master.hpp>

namespace _NS_UTILITY {
	enum class AES_KeyLength {
		AES_KEY_128 = 0,
		AES_KEY_256 = 1
	};

	const int AES_Keylen2NumByte[] = { 128/8, 256/8 };
}
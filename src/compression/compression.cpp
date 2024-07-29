#include <igcclib/extern/miniz.h>
#include <igcclib/compression/compression.hpp>

namespace _NS_UTILITY {
	std::vector<uint8_t> compress_data(const uint8_t* in_data, size_t n)
	{
		auto outlen = mz_compressBound(n);
		std::vector<uint8_t> output(outlen);

		auto is_ok = mz_compress(output.data(), &outlen, in_data, n);
		assert_throw(is_ok == MZ_OK, "failed to compress file");

		output.resize(outlen);
		return output;
	}


	std::vector<uint8_t> decompress_data(const uint8_t* in_data, size_t n_input, size_t n_output)
	{
		std::vector<uint8_t> output(n_output);
		mz_ulong outlen = n_output;
		auto is_ok = mz_uncompress(output.data(), &outlen, in_data, n_input);
		assert_throw(is_ok == MZ_OK, "failed to decompress file");

		output.resize(outlen);
		return output;
	}
}
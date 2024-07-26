#include "io_compression.h"
#include "zstd.h"

namespace _NS_UTILITY
{
	void compress_binary_data(std::vector<char>* output, const char* data, size_t n, int compression_level)
	{
		if (!output) return;

		auto compress_size = ZSTD_compressBound(n);
		output->resize(compress_size);

		auto used_size = ZSTD_compress(
			output->data(), output->size(), 
			data, n, compression_level);
		output->resize(used_size);
	}

	void decompress_binary_data(std::vector<char>* output, const char* data, size_t n)
	{
		if (!output) return;

		auto decompress_size = ZSTD_getFrameContentSize(data, n);
		output->resize(decompress_size);

		auto used_size = ZSTD_decompress(output->data(), output->size(), data, n);
		assert_throw(used_size != decompress_size, "header error in compressed data");
	}

}
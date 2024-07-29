#include <openssl/rand.h>
#include <openssl/aes.h>
#include <openssl/evp.h>
#include <igcclib/crypto/AES_ECB_Cipher.hpp>
#include <igcclib/crypto/igcclib_openssl.hpp>

namespace _NS_UTILITY {
	static const EVP_CIPHER* get_aes_ecb_cipher(AES_KeyLength keylen) {
		if (keylen == AES_KeyLength::AES_KEY_128)
			return EVP_aes_128_ecb();
		else if (keylen == AES_KeyLength::AES_KEY_256)
			return EVP_aes_256_ecb();
		else
			return NULL;
	}

	using EVP_CTX_ptr = std::shared_ptr<EVP_CIPHER_CTX>;

	void AES_ECB_Cipher::init_with_random_key(AES_KeyLength keylen)
	{
		int n = AES_Keylen2NumByte[(int)keylen];
		std::vector<uint8_t> key(n);
		RAND_bytes(key.data(), key.size());

		std::string _key(key.begin(), key.end());
		init_with_key(_key, keylen);
	}

	void AES_ECB_Cipher::init_with_key(const std::string& key, AES_KeyLength keylen)
	{
		int n = AES_Keylen2NumByte[(int)keylen];
		if (key.size() > n)
			assert_throw(false, "key is longer than limit");
		m_key = key;

		std::string key_ext(n, 0);
		for (int i = 0; i < key.size(); i++)
			key_ext[i] = key[i];
		m_key_ext = key_ext;
		m_keylen = keylen;
	}

	size_t AES_ECB_Cipher::get_padded_plain_text_len(size_t len) const
	{
		return (len / AES_BLOCK_SIZE + 1)*AES_BLOCK_SIZE;
	}

	void AES_ECB_Cipher::encrypt(std::vector<uint8_t>& output, const std::string& plain_text)
	{
		encrypt(output, (const uint8_t*)plain_text.c_str(), plain_text.size());
	}

	void AES_ECB_Cipher::encrypt(std::vector<uint8_t>& output, const uint8_t* plain_text, int n)
	{
		EVP_CTX_ptr ctx(EVP_CIPHER_CTX_new(), ::EVP_CIPHER_CTX_free);

		//init encryption
		{
			int res = EVP_EncryptInit_ex(ctx.get(), get_aes_ecb_cipher(m_keylen), NULL, (const uint8_t*)m_key_ext.c_str(), NULL);
			if (res == 0)
				assert_throw(false, get_openssl_error());
		}
		
		//prepare buffer
		auto buf_size = get_padded_plain_text_len(n);
		std::vector<uint8_t> enc(buf_size);

		//perform encryption
		{
			int enclen = 0;
			int res = EVP_EncryptUpdate(ctx.get(), enc.data(), &enclen, plain_text, n);
			if (res == 0)
				assert_throw(false, get_openssl_error());

			int enclen_final = 0;
			int res_final = EVP_EncryptFinal_ex(ctx.get(), enc.data() + enclen, &enclen_final);
			if (res_final == 0)
				assert_throw(false, get_openssl_error());

			//only keep valid data
			output.assign(enc.begin(), enc.begin() + enclen + enclen_final);
		}
	}

	void AES_ECB_Cipher::decrypt(std::vector<uint8_t>& output, const std::string& cipher_text)
	{
		decrypt(output, (const uint8_t*)cipher_text.c_str(), cipher_text.size());
	}

	void AES_ECB_Cipher::decrypt(std::vector<uint8_t>& output, const uint8_t* cipher_text, int n)
	{
		EVP_CTX_ptr ctx(EVP_CIPHER_CTX_new(), ::EVP_CIPHER_CTX_free);

		//init decryption
		{
			int res = EVP_DecryptInit_ex(ctx.get(), get_aes_ecb_cipher(m_keylen), NULL, (const uint8_t*)m_key_ext.c_str(), NULL);
			if (res == 0)
				assert_throw(false, get_openssl_error());
		}

		//prepare buffer
		auto buf_size = get_padded_plain_text_len(n);
		std::vector<uint8_t> dec(buf_size);

		//perform decryption
		{
			int declen = 0;
			int res = EVP_DecryptUpdate(ctx.get(), dec.data(), &declen, cipher_text, n);
			if (res == 0)
				assert_throw(false, get_openssl_error());

			int declen_final = 0;
			int res_final = EVP_DecryptFinal_ex(ctx.get(), dec.data() + declen, &declen_final);
			if (res_final == 0)
				assert_throw(false, get_openssl_error());

			//only keep valid data
			output.assign(dec.begin(), dec.begin() + declen + declen_final);
		}
	}
}
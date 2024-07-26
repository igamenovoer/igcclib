#include "openssl/rsa.h"
#include "openssl/bio.h"
#include "openssl/bn.h"
#include "openssl/pem.h"
#include "openssl/err.h"
#include <igcclib/crypto/RSA_Cipher.hpp>
#include <igcclib/crypto/igcclib_openssl.hpp>

namespace _NS_UTILITY {
	using BN_ptr = std::shared_ptr<BIGNUM>;
	using BIO_MEM_ptr = std::shared_ptr<BIO>;

	static const int RSA_PAD_METHOD[] = {RSA_NO_PADDING, RSA_PKCS1_PADDING, RSA_PKCS1_OAEP_PADDING};

	RSA_Cipher::RSA_Cipher()
	{
		m_rsa = std::shared_ptr<RSA>(RSA_new(), ::RSA_free);
	}

	void RSA_Cipher::init_with_random_key(int nbit) {
		auto rsa_obj = std::shared_ptr<RSA>(RSA_new(), ::RSA_free);
		m_rsa = rsa_obj;

		BN_ptr bne(BN_new(), ::BN_free);
		BN_set_word(bne.get(), RSA_F4);
		RSA_generate_key_ex(rsa_obj.get(), nbit, bne.get(), NULL);
	}

	void RSA_Cipher::init_with_public_key_pem(const std::string& key_text) {
		auto rsa_obj = std::shared_ptr<RSA>();

		BIO_MEM_ptr bio(BIO_new_mem_buf(key_text.c_str(), (int)key_text.size()), ::BIO_free_all);

		//check key format
		if (key_text.find("BEGIN PUBLIC KEY") != key_text.npos) {
			//pkcs8 format
			RSA* p = NULL;
			PEM_read_bio_RSA_PUBKEY(bio.get(), &p, NULL, NULL);
			rsa_obj.reset(p, ::RSA_free);

		}
		else if (key_text.find("BEGIN RSA PUBLIC KEY") != key_text.npos) {
			//pkcs1 format
			RSA* p = NULL;
			PEM_read_bio_RSAPublicKey(bio.get(), &p, NULL, NULL);
			rsa_obj.reset(p, ::RSA_free);
		}
		else {
			assert_throw(false, "unknown public key format");
		}

		m_rsa = rsa_obj;
	}

	void RSA_Cipher::init_with_private_key_pem(const std::string& key_text) {
		auto rsa_obj = std::shared_ptr<RSA>(RSA_new(), ::RSA_free);
		BIO_MEM_ptr bio(BIO_new_mem_buf(key_text.c_str(), (int)key_text.size()), ::BIO_free_all);

		//check key format
		if (key_text.find("BEGIN RSA PRIVATE KEY") != key_text.npos) {
			//pkcs1 format
			RSA* p = NULL;
			PEM_read_bio_RSAPrivateKey(bio.get(), &p, NULL, NULL);
			rsa_obj.reset(p, ::RSA_free);
		}
		else {
			assert_throw(false, "unknown private key format");
		}
		m_rsa = rsa_obj;
	}

	void RSA_Cipher::encrypt_public(std::vector<uint8_t>& output, const uint8_t* plain_text, size_t n)
	{
		auto rsa_obj = std::static_pointer_cast<RSA>(m_rsa);
		if (rsa_obj == nullptr)
			assert_throw(false, "not initialized yet");
		
		//create buffer
		auto buf_size = RSA_size(rsa_obj.get());
		output.resize(buf_size);

		int pad_type = RSA_PAD_METHOD[(int)m_pad_type];
		int len = RSA_public_encrypt(n, plain_text, output.data(), rsa_obj.get(), pad_type);
		if (len == -1)
			assert_throw(false, get_openssl_error());
		output.resize(len);
	}

	void RSA_Cipher::decrypt_public(std::vector<uint8_t>& output, const uint8_t* cipher_text, size_t n)
	{
		auto rsa_obj = std::static_pointer_cast<RSA>(m_rsa);
		if (rsa_obj == nullptr)
			assert_throw(false, "not initialized yet");

		//create buffer
		auto buf_size = RSA_size(rsa_obj.get());
		output.resize(buf_size);

		int pad_type = RSA_PAD_METHOD[(int)m_pad_type];
		int len = RSA_public_decrypt(n, cipher_text, output.data(), rsa_obj.get(), pad_type);
		if (len == -1)
			assert_throw(false, get_openssl_error());
		output.resize(len);
	}

	void RSA_Cipher::encrypt_private(std::vector<uint8_t>& output, const uint8_t* plain_text, size_t n)
	{
		auto rsa_obj = std::static_pointer_cast<RSA>(m_rsa);
		if (rsa_obj == nullptr)
			assert_throw(false, "not initialized yet");

		//create buffer
		auto buf_size = RSA_size(rsa_obj.get());
		output.resize(buf_size);

		int pad_type = RSA_PAD_METHOD[(int)m_pad_type];
		int len = RSA_private_encrypt(n, plain_text, output.data(), rsa_obj.get(), pad_type);
		if (len == -1)
			assert_throw(false, get_openssl_error());
		output.resize(len);
	}

	void RSA_Cipher::decrypt_private(std::vector<uint8_t>& output, const uint8_t* cipher_text, size_t n)
	{
		auto rsa_obj = std::static_pointer_cast<RSA>(m_rsa);
		if (rsa_obj == nullptr)
			assert_throw(false, "not initialized yet");

		//create buffer
		auto buf_size = RSA_size(rsa_obj.get());
		output.resize(buf_size);

		int pad_type = RSA_PAD_METHOD[(int)m_pad_type];
		int len = RSA_private_decrypt(n, cipher_text, output.data(), rsa_obj.get(), pad_type);
		if (len == -1)
			assert_throw(false, get_openssl_error());
		output.resize(len);
	}

	size_t RSA_Cipher::get_max_plain_text_len() const
	{
		auto rsa_obj = std::static_pointer_cast<RSA>(m_rsa);
		if (rsa_obj == nullptr)
			assert_throw(false, "not initialized yet");
		auto buf_size = RSA_size(rsa_obj.get());
		return buf_size;
	}

	void RSA_Cipher::encrypt_public(std::vector<uint8_t>& output, const std::string& plain_text)
	{
		encrypt_public(output, (const uint8_t*)plain_text.c_str(), plain_text.size());
	}

	void RSA_Cipher::decrypt_public(std::vector<uint8_t>& output, const std::string& cipher_text)
	{
		decrypt_public(output, (const uint8_t*)cipher_text.c_str(), cipher_text.size());
	}

	void RSA_Cipher::encrypt_private(std::vector<uint8_t>& output, const std::string& plain_text)
	{
		encrypt_private(output, (const uint8_t*)plain_text.c_str(), plain_text.size());
	}

	void RSA_Cipher::decrypt_private(std::vector<uint8_t>& output, const std::string& cipher_text)
	{
		decrypt_private(output, (const uint8_t*)cipher_text.c_str(), cipher_text.size());
	}
}
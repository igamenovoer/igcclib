#pragma once

#include <igcclib/crypto/igcclib_crypto_def.hpp>

namespace _NS_UTILITY {

	/*!
	 * \class AES cipher using ECB mode.
	 *
	 * \brief PKCS5 padding is used in all encryption/decryption.
	 */
	class AES_ECB_Cipher {
	protected:
		std::string m_key; //the key in plain text
		std::string m_key_ext; //the padded key
		AES_KeyLength m_keylen;

	public:
		~AES_ECB_Cipher() {}

		/**
		* \brief initialize by generating random key
		*
		* \param keylen length of the aes key
		*/
		void init_with_random_key(AES_KeyLength keylen);

		/**
		* \brief initialize by a given key
		*
		* \param key the key, must be shorter than the size specified in keylen. 
		* If the key is not long enough, it will be padded with 0s.
		* \param keylen the aes key length
		*/
		void init_with_key(const std::string& key, AES_KeyLength keylen);

		/** \brief get the input key */
		const std::string& get_key() const { return m_key; }

		/**
		* \brief the length of the padded plain text
		*
		* \param len the input plain text length
		* \return size_t the padded text length
		*/
		size_t get_padded_plain_text_len(size_t len) const;

		/**
		* \brief encrypt plain_text using aes key
		*
		* \param output the encrypted data
		* \param plain_text the input plain_text
		* \param n length of the plain_text
		*/
		void encrypt(std::vector<uint8_t>& output, const uint8_t* plain_text, int n);

		/**
		* \brief encrypt plain_text using aes key
		*
		* \param output the encrypted data
		* \param plain_text the input plain_text
		*/
		void encrypt(std::vector<uint8_t>& output, const std::string& plain_text);

		/**
		* \brief decrypt the cipher text using aes key
		*
		* \param output the decrypted data
		* \param cipher_text the input encrypted data
		* \param n length of the cipher_text
		*/
		void decrypt(std::vector<uint8_t>& output, const uint8_t* cipher_text, int n);

		/**
		* \brief decrypt the cipher text using aes key
		*
		* \param output the decrypted data
		* \param cipher_text the input encrypted data
		*/
		void decrypt(std::vector<uint8_t>& output, const std::string& cipher_text);
	};
}
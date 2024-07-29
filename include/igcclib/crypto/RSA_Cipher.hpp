#pragma once

#include <igcclib/igcclib_master.hpp>

namespace _NS_UTILITY {
	enum class RSA_Padding {
		NONE = 0,
		PKCS1 = 1,
		PKCS1_OEAP = 2
	};

	//RSA encryption and decryption using openssl
	class IGCCLIB_API RSA_Cipher {
	protected:
		std::shared_ptr<void> m_rsa; //openssl rsa data, we want to avoid the openssl headers here
		RSA_Padding m_pad_type = RSA_Padding::PKCS1;
	public:
		RSA_Cipher();
		virtual ~RSA_Cipher() {}

		/**
		* \brief initialize RSA by generating nbit keys
		*
		* \param nbit number of bits of the keys, must be no less than 1024		
		*/
		void init_with_random_key(int nbit = 1024);

		/**
		* \brief initialize RSA with a pem-encoded public key.
		*
		* After initialization, public-key functions are usable
		*
		* \param key_text the pem encoded public key. Can be pkcs1 or pkcs8 format
		*/
		void init_with_public_key_pem(const std::string& key_text);

		/**
		* \brief initialize RSA with a pem-encoded private key.
		*
		* After initialization, private-key functions are usable
		*
		* \param key_text the pem encoded private key. Can be pkcs1 or pkcs8 format
		*/
		void init_with_private_key_pem(const std::string& key_text);

		/**
		* \brief specify the padding type for the rsa encryption and decryption
		*
		* \param pad_type the padding type
		*/
		void set_padding_type(RSA_Padding pad_type) { m_pad_type = pad_type; }
		
		/**
		* \brief get the padding type
		*
		* \return RSA_Padding the paddin type
		*/
		RSA_Padding get_padding_type() const { return m_pad_type; }

		/** \brief the maximum length of encryptable plain text */
		size_t get_max_plain_text_len() const;

		/**
		* \brief encrypt plain_text using public key
		*
		* \param output the encrypted data
		* \param plain_text the input plain_text
		* \param n length of the plain_text
		*/
		void encrypt_public(std::vector<uint8_t>& output, const uint8_t* plain_text, size_t n);

		/**
		* \brief encrypt plain_text using public key
		*
		* \param output the encrypted data
		* \param plain_text the input plain_text
		*/
		void encrypt_public(std::vector<uint8_t>& output, const std::string& plain_text);

		/**
		* \brief decrypt the cipher text using public key
		*
		* \param output the decrypted data
		* \param cipher_text the input encrypted data
		* \param n length of the cipher_text
		*/
		void decrypt_public(std::vector<uint8_t>& output, const uint8_t* cipher_text, size_t n);

		/**
		* \brief decrypt the cipher text using public key
		*
		* \param output the decrypted data
		* \param cipher_text the input encrypted data
		*/
		void decrypt_public(std::vector<uint8_t>& output, const std::string& cipher_text);

		/**
		* \brief encrypt plain_text using private key
		*
		* \param output the encrypted data
		* \param plain_text the input plain_text
		* \param n length of the plain_text
		*/
		void encrypt_private(std::vector<uint8_t>& output, const uint8_t* plain_text, size_t n);

		/**
		* \brief encrypt plain_text using private key
		*
		* \param output the encrypted data
		* \param plain_text the input plain_text
		*/
		void encrypt_private(std::vector<uint8_t>& output, const std::string& plain_text);

		/**
		* \brief decrypt the cipher text using private key
		*
		* \param output the decrypted data
		* \param cipher_text the input encrypted data
		* \param n length of the cipher_text
		*/
		void decrypt_private(std::vector<uint8_t>& output, const uint8_t* cipher_text, size_t n);

		/**
		* \brief decrypt the cipher text using private key
		*
		* \param output the decrypted data
		* \param cipher_text the input encrypted data
		*/
		void decrypt_private(std::vector<uint8_t>& output, const std::string& cipher_text);
	};
};
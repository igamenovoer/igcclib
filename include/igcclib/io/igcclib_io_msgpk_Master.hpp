#pragma once

//MessagePack serialization and RPC support
//requires rpclib: https://github.com/rpclib/rpclib

#include <igcclib/igcclib_master.hpp>
#include <msgpack.hpp>
#include <msgpack/adaptor/vector.hpp>
#include <msgpack/adaptor/msgpack_tuple.hpp>

#define _MSGPK_CUSTOM_ADAPTOR_BEGIN \
	namespace msgpack { \
	using msgobj = object; \
	MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) { \
		namespace adaptor { \

#define _MSGPK_CUSTOM_ADAPTOR_END }}};
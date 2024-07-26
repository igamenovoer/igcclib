#pragma once

//MessagePack serialization and RPC support
//requires rpclib: https://github.com/rpclib/rpclib

#include "./../common/def_master.h"

#include "rpc/msgpack.hpp"
#include "rpc/msgpack/adaptor/vector.hpp"
#include "rpc/msgpack/adaptor/msgpack_tuple.hpp"

#define _MSGPK_CUSTOM_ADAPTOR_BEGIN \
	namespace clmdep_msgpack { \
	using msgobj = object; \
	MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) { \
		namespace adaptor { \

#define _MSGPK_CUSTOM_ADAPTOR_END }}};
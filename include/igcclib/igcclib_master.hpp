#pragma once

#ifndef _NS_UTILITY
#define _NS_UTILITY igcclib
#endif

#ifndef _FLOAT_TYPE
	#define _FLOAT_TYPE double
#endif

#ifndef _INT_TYPE
	#define _INT_TYPE int
#endif

#include <string>
#include <memory>
#include <functional>
#include <type_traits>
#include <atomic>

#include <igcclib/igcclib_api.h>

//you need to specifically turn off assert print if you want
#ifndef _ASSERT_NO_PRINT
	#define _ASSERT_WITH_PRINT
#endif

#ifdef _ASSERT_WITH_PRINT
	#include <iostream>
#endif

#ifdef _ASSERT_WITH_BACKTRACE
	#define NOMINMAX
	#include <boost/stacktrace.hpp>
#endif


#ifdef _ENABLE_CEREAL_SUPPORT
	#include <cereal/cereal.hpp>
	#include <cereal/types/vector.hpp>
	#include <cereal/types/string.hpp>
	#include <cereal/types/memory.hpp>
	#include <cereal/types/array.hpp>
	#include <cereal/types/common.hpp>
	#include <cereal/types/tuple.hpp>
	#include <cereal/types/map.hpp>
	#include <cereal/types/set.hpp>
	#include <cereal/archives/binary.hpp>
	#include <cereal/archives/portable_binary.hpp>
	#include <cereal/types/base_class.hpp>
	#include <cereal/types/utility.hpp>
	#include <cereal/types/polymorphic.hpp>
#endif

namespace _NS_UTILITY
{
	using float_type = _FLOAT_TYPE;
	using int_type = _INT_TYPE;
	using uint_type = unsigned _INT_TYPE;

	typedef std::string GENERIC_ERROR;
	using GeneralException = std::logic_error;

	namespace MathConstant {
		constexpr double PI = 3.14159265359;
		constexpr double Deg2Rad = PI / 180.0;
		constexpr double Rad2Deg = 180.0 / PI;
		constexpr double GravityMagnitude = 9.80665;	//g in m/s^2
	}

	enum class VideoCodecType
	{
		LIBX264_RGB,
		LIBX264_YUV,
		H264_NVENC,
		H265_NVENC,
		H264_INTEL_QUICK_SYNC,
		FFV1
	};

	namespace VideoCodecName
	{
		const std::string LIBX264 = "libx264";
		const std::string LIBX264_RGB = "libx264rgb";
		const std::string H264_NVIDIA = "h264_nvenc";
		const std::string H265_NVIDIA = "hevc_nvenc";
		const std::string H264_QUICK_SYNC = "h264_qsv";
		const std::string MJPEG_OPENCV = "mjpeg_opencv";
		const std::string MJPEG_FFMPEG = "mjpeg";
		const std::string FFV1 = "ffv1";
	}
	
	/**
	* \brief when condition is not satisified, throw an exception or print out a warning
	*
	* \param expr	when a condition is satisfied, if false, triggers exception or warning.
	* \param msg		the message to print when expr is false
	* \param warn_only	if true, only print out the error message without throwing exception
	*/
	inline void assert_throw(bool expr, std::string msg, bool warn_only = false)
	{
		if (!expr)
		{
#ifdef _ASSERT_WITH_PRINT
			std::cout << "== ASSERT FAILED ==" << std::endl;
			std::cout << msg << std::endl;
			std::cout.flush();

	#ifdef _ASSERT_WITH_BACKTRACE
			std::cout << boost::stacktrace::stacktrace() << std::endl;
	#endif
#endif
			if(!warn_only)
				throw GeneralException(msg.c_str());
		}
	}

	/// <summary>
	/// This type of object is considered heavy, and therefore using = to copy
	/// will only copy as a reference, you must call clone() to truely copy the object.
	/// </summary>
	template<typename T>
	class HeavyObject
	{
	public:
		virtual void clone(T&) const = 0;
		virtual T& operator= (const T&) = delete; //disable assignment
		HeavyObject(const HeavyObject&) = delete; //disable copy constructor
		HeavyObject() = default;
	};

	//class supporting expilcit copy
	template<typename T>
	class IExplicitCopyable {
	public:
		virtual void copy_to(T& dst) const = 0;
	};

	/// <summary>
	/// use this function to initialize a shared_ptr without specifying its data type.
	/// </summary>
	template<typename T>
	void init_ptr_with_default_constructor(std::shared_ptr<T>& ptr)
	{
		ptr = std::make_shared<T>();
	}

	/// <summary>
	/// Explicit image format, useful with opencv
	/// </summary>
	enum class ImageFormat
	{
		//DO NOT CHANGE THEIR INTEGER VALUES!!

		NONE = 0, //special value representing "do not care"

		GRAY = 1,
		RGB = 2,
		RGBA = 3,
		BGR = 4,
		BGRA = 5
	};
	
	/// <summary>
	/// get number of channels for image format
	/// </summary>
	/// <param name="fmt">the image format</param>
	/// <returns>number of channels</returns>
	inline int get_num_channel(ImageFormat fmt)
	{
		int n = 0;
		switch (fmt) {
		case ImageFormat::BGR:
		case ImageFormat::RGB:
			n = 3;
			break;
		case ImageFormat::BGRA:
		case ImageFormat::RGBA:
			n = 4;
			break;
		case ImageFormat::GRAY:
			n = 1;
			break;
		default:
			break;
		}
		return n;
	}

	//vertex order used in formulation
	enum class VertexOrderInFormulation
	{
		NONE, //unspecified vertex order
		XXYYZZ, //the output vertex order is (x1,x2,...,y1,y2,...z1,z2)
		XYZXYZ  //the output vertex order is (x1,y1,z1,x2,y2,z2,...)
	};

	//represent a partial copy-on-write object
	//where the underlying object is linked to its source, only break
	//the link when assigned onto.
	//LinkedObject itself only keeps a reference to the underlying object.
	template<typename T, bool = std::is_base_of<HeavyObject<T>, T>::value>
	struct LinkedObject {};

	template<typename T>
	class LinkedObjectBase {
	protected:
		std::shared_ptr<T> data;
		std::atomic<bool> is_linked{false};
		std::function<T*(void)> obj_constructor;

	public:
		using DataType = T;

		LinkedObjectBase(bool create_data, std::function<T*(void)> data_constructor) {
			obj_constructor = data_constructor;
			if (create_data) {
				data.reset(obj_constructor());
			}
		}

		LinkedObjectBase(bool create_data = false): 
			LinkedObjectBase(create_data, []() {return new T(); }) {}

		LinkedObjectBase(const LinkedObjectBase& obj) {
			data = obj.data;
			obj_constructor = obj.obj_constructor;
			is_linked = obj.is_linked.load();
		}

		LinkedObjectBase& operator = (const LinkedObjectBase& rhs) {
			data = rhs.data;
			obj_constructor = rhs.obj_constructor;
			is_linked = rhs.is_linked.load();
			return *this;
		}

		~LinkedObjectBase() {}

		/// <summary>
		/// assign new data into this object, linking to the source.
		/// Will NOT link if the input is nullptr
		/// </summary>
		/// <param name="_data">the new data</param>
		virtual void assign_linked(std::shared_ptr<T> _data) {
			data = _data;
			is_linked = data != nullptr;
		}

		/// <summary>
		/// assign new data into this object, linking to the source.
		/// Will NOT link if the input is nullptr
		/// </summary>
		/// <param name="_obj">the new data</param>
		virtual void assign_linked(const LinkedObjectBase<T>& _obj) {
			data = _obj.data;
			is_linked = data != nullptr;
		}

		//get the underlying object
		virtual T& object() { return *data; }
		virtual const T& object() const { return *data; }
		virtual std::shared_ptr<T> object_ptr() const { return data; }
		virtual bool linked() const { return is_linked; }

		virtual void assign_copy(const T& _data, bool break_link = true) = 0;
		virtual void assign_copy(const LinkedObjectBase<T>& _obj, bool break_link = true) {
			if (_obj.data == nullptr)
			{
				data.reset();
				is_linked = false;
			}
			else
				assign_copy(_obj.object(), break_link);
		}

		/// <summary>
		/// break the link, making the data independent of the source
		/// </summary>
		virtual void make_independent() {
			if (!is_linked)
				return;

			if (data != nullptr)
				assign_copy(*data, true);

			is_linked = false;
		}
	};

	//for classes not inheriting from HeavyObject
	template<typename T>
	struct LinkedObject<T, false>: public LinkedObjectBase<T> {
		using LinkedObjectBase<T>::assign_copy;
		using LinkedObjectBase<T>::LinkedObjectBase;
		//using LinkedObjectBase<T>::operator =;

		/// <summary>
		/// assign new data into this object, breaking the link by default
		/// </summary>
		/// <param name="_data">the new data which will be copied</param>
		/// <param name="break_link">it true, the link to source will be broken</param>
		void assign_copy(const T& _data, bool break_link = true) override {
			auto tmp = this->data; //keep the data alive, in case _data is actually *data

			if (this->data == nullptr || (break_link && this->is_linked)) {
				this->data.reset(this->obj_constructor());
				this->is_linked = false;
			}

			*this->data = _data;
		}
	};

	//for classes inheriting from HeavyObject
	template<typename T>
	struct LinkedObject<T, true>: public LinkedObjectBase<T> {
		using LinkedObjectBase<T>::assign_copy;
		using LinkedObjectBase<T>::LinkedObjectBase;
		//using LinkedObjectBase<T>::operator =;

		/// <summary>
		/// assign new data into this object, breaking the link by default
		/// </summary>
		/// <param name="_data">the new data which will be copied</param>
		/// <param name="break_link">it true, the link to source will be broken</param>
		void assign_copy(const T& _data, bool break_link = true) override {
			auto tmp = this->data; //keep the data alive, in case _data is actually *data

			if (this->data == nullptr || (break_link && this->is_linked)) {
				this->data.reset(this->obj_constructor());
				this->is_linked = false;
			}
			_data.clone(*this->data);
		}		
	};
};

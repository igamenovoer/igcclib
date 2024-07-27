#pragma once
#include <igcclib/vision/igcclib_opencv_def.hpp>

namespace _NS_UTILITY
{
	class IGCCLIB_API ImageData {
	public:
		cv::Mat data;
		ImageFormat format = ImageFormat::NONE;

		/** \brief copy the whole image data */
		void copy_to(ImageData& output) const {
			data.copyTo(output.data);
			output.format = format;
		}

		/** \brief copy the cvmat */
		void copy_to(cv::Mat& output) const {
			data.copyTo(output);
		}

		ImageData() {}
		ImageData(const cv::Mat& img, ImageFormat format)
		{
			data = img;
			this->format = format;
		}

		void convert_to(ImageData& output, ImageFormat to_format) const
		{
			static std::map<std::pair<ImageFormat, ImageFormat>, int> flagdict{
				{ { ImageFormat::BGR, ImageFormat::BGRA }, cv::COLOR_BGR2BGRA },
				{ { ImageFormat::BGR, ImageFormat::RGB }, cv::COLOR_BGR2RGB },
				{ { ImageFormat::BGR, ImageFormat::RGBA }, cv::COLOR_BGR2RGBA },
				{ { ImageFormat::BGR, ImageFormat::GRAY }, cv::COLOR_BGR2GRAY },

				{ { ImageFormat::BGRA, ImageFormat::BGR }, cv::COLOR_BGRA2BGR },
				{ { ImageFormat::BGRA, ImageFormat::RGB }, cv::COLOR_BGRA2RGB },
				{ { ImageFormat::BGRA, ImageFormat::RGBA }, cv::COLOR_BGRA2RGBA },
				{ { ImageFormat::BGRA, ImageFormat::GRAY }, cv::COLOR_BGRA2GRAY },

				{ { ImageFormat::RGB, ImageFormat::BGR }, cv::COLOR_RGB2BGR },
				{ { ImageFormat::RGB, ImageFormat::BGRA }, cv::COLOR_RGB2BGRA },
				{ { ImageFormat::RGB, ImageFormat::RGBA }, cv::COLOR_RGB2RGBA },
				{ { ImageFormat::RGB, ImageFormat::GRAY }, cv::COLOR_RGB2GRAY },

				{ { ImageFormat::RGBA, ImageFormat::BGR }, cv::COLOR_RGBA2BGR },
				{ { ImageFormat::RGBA, ImageFormat::BGRA }, cv::COLOR_RGBA2BGRA },
				{ { ImageFormat::RGBA, ImageFormat::RGB }, cv::COLOR_RGBA2RGB },
				{ { ImageFormat::RGBA, ImageFormat::GRAY }, cv::COLOR_RGBA2GRAY },

				{ { ImageFormat::GRAY, ImageFormat::BGR }, cv::COLOR_GRAY2BGR },
				{ { ImageFormat::GRAY, ImageFormat::BGRA }, cv::COLOR_GRAY2BGRA },
				{ { ImageFormat::GRAY, ImageFormat::RGB }, cv::COLOR_GRAY2RGB },
				{ { ImageFormat::GRAY, ImageFormat::RGBA }, cv::COLOR_GRAY2RGBA },
			};

			if (format == to_format) {
				copy_to(output);
			}
			else {
				auto cvtcode = flagdict[{format, to_format}];
				cv::cvtColor(this->data, output.data, cvtcode);
				output.format = to_format;
			}
		}
		bool is_empty() const {
			return data.empty();
		}

		//cereal support
		template<typename T>
		void serialize(T& ar) {
			ar(data);
			ar(format);
		}
	};
}

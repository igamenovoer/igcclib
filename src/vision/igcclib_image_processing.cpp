#include <numeric>
#include <cmath>

#include <igcclib/vision/igcclib_image_processing.hpp>

namespace _NS_UTILITY {
	void rotate_image(cv::Mat& output, fMATRIX_3* out_transmat, 
		const cv::Mat& input_image, double angle_rad, bool preserve_image_size /*= false*/)
	{
		auto height = input_image.rows;
		auto width = input_image.cols;

		//compose transformation matrix
		if(!preserve_image_size)
		{
			fMATRIX corner_points(2, 4); //each column is a point
			corner_points.col(0) = fVECTOR_2(0, 0);
			corner_points.col(1) = fVECTOR_2(0, height);
			corner_points.col(2) = fVECTOR_2(width, height);
			corner_points.col(3) = fVECTOR_2(width, 0);

			//image axis is x+ right, y+ down, to rotate in ccw direction relative to screen
			//we should actually rotate -angle
			auto rot = Eigen::Rotation2D<float_type>(-angle_rad);
			fMATRIX rot_pts = rot.matrix() * corner_points;
			
			//find bounding box
			fVECTOR_2 minc = rot_pts.rowwise().minCoeff();
			fVECTOR_2 maxc = rot_pts.rowwise().maxCoeff();
			fVECTOR_2 boxlen = maxc - minc;

			//compute shift, to shift the transformed image so that after rotation its corner is at (0,0)
			auto shift = Eigen::Translation<float_type, 2>(-minc[0], -minc[1]);

			//compose the matrix
			fMATRIX_3 tmat = (shift * rot).matrix();

			cv::Mat cv_transmat(3, 3, cv::DataType<float_type>::type, tmat.data());
			cv::Size newsize((int)boxlen[0], (int)boxlen[1]);

			cv::warpPerspective(input_image, output, cv_transmat, newsize);

			if (out_transmat)
				*out_transmat = tmat.transpose();
		}
		else {
			cv::Point2f center(float(width / 2.0), float(height / 2.0));

			auto pi = std::acos(-1.0);
			auto rotmat = cv::getRotationMatrix2D(center, angle_rad / pi * 180, 1.0);
			cv::warpAffine(input_image, output, rotmat, cv::Size(input_image.cols, input_image.rows));

			if (out_transmat)
			{
				out_transmat->setIdentity();
				rotmat.convertTo(rotmat, cv::DataType<float_type>::type);
				out_transmat->block(0, 0, 2, 3) = Eigen::Map<fMATRIX>((float_type*)rotmat.data, 2, 3);
				out_transmat->transposeInPlace();
			}
		}
	}

	void scale_pixel_rect(iVECTOR_4& inout_xywh, int width, int height, double scale, bool force_square /*= false*/)
	{
		double x = inout_xywh(0);
		double y = inout_xywh(1);
		double w = inout_xywh(2);
		double h = inout_xywh(3);

		//center
		double cx = x + w / 2;
		double cy = y + h / 2;

		//make square?
		if (force_square)
		{
			double edge_length = std::min({ w,h });
			w = edge_length;
			h = edge_length;
			x = cx - w / 2;
			y = cx - h / 2;
		}

		//compute the scale such that it does not exceed the image boundary
		double x_scale_max = 2 * std::min({ cx, width - cx - 1 }) / w;
		double y_scale_max = 2 * std::min({ cy, height - cy - 1 }) / h;
		double scale_max = std::min({ x_scale_max, y_scale_max });

		//bound the intended scale
		scale = std::min({ scale, scale_max });

		//scale the pixel box
		w = scale * w;
		h = scale * h;

		//compute the left top
		x = cx - w / 2;
		y = cy - h / 2;

		inout_xywh(0) = (int_type)std::round(x);
		inout_xywh(1) = (int_type)std::round(y);
		inout_xywh(2) = (int_type)std::round(w);
		inout_xywh(3) = (int_type)std::round(h);
	}

	void convert_image(const cv::Mat& img, cv::Mat& output, ImageFormat fmt_from, ImageFormat fmt_to, bool auto_adjust_format /*= false*/)
	{
		if (fmt_from == fmt_to)
		{
			img.copyTo(output);
			return;
		}

		if (auto_adjust_format)
		{
			if (img.channels() == 1) //a single channel image can only be grayscale
				fmt_from = ImageFormat::GRAY;
			else if (img.channels() == 3) //remove alpha channel because the image has only 3 channels
				switch (fmt_from) {
				case ImageFormat::BGRA: fmt_from = ImageFormat::BGR; break;
				case ImageFormat::RGBA: fmt_from = ImageFormat::RGB; break;
				}
			else if (img.channels() == 4) //add alpha channel
				switch (fmt_from) {
				case ImageFormat::BGR: fmt_from = ImageFormat::BGRA; break;
				case ImageFormat::RGB: fmt_from = ImageFormat::RGBA; break;
				}
		}

		assert_throw(fmt_from != ImageFormat::NONE && fmt_to != ImageFormat::NONE,
			"image format cannot be NONE");

		static std::map<std::pair<ImageFormat, ImageFormat>, int> flagdict{
			{
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
			}
		};

		int code = flagdict[{fmt_from, fmt_to}];
		cv::cvtColor(img, output, code);
	}

	void imread(const std::string filename, cv::Mat& output, ImageFormat output_format /*= ImageFormat::NONE*/)
	{
		output = cv::imread(filename, cv::IMREAD_UNCHANGED);
		ImageFormat fmt_input = ImageFormat::NONE;
		switch (output.channels())
		{
		case 1:
			fmt_input = ImageFormat::GRAY;
			break;
		case 3:
			fmt_input = ImageFormat::BGR;
			break;
		case 4:
			fmt_input = ImageFormat::BGRA;
			break;
		}
		if (output_format != ImageFormat::NONE && output_format != fmt_input)
			convert_image(output, output, fmt_input, output_format);
	}

	void imwrite(const std::string filename, const cv::Mat& image, ImageFormat format /*= ImageFormat::NONE*/)
	{
		if (format == ImageFormat::NONE)
		{
			cv::imwrite(filename, image);
			return;
		}

		cv::Mat output;
		if (image.channels() == 1)
			output = image;
		else if (image.channels() == 3)
			convert_image(image, output, format, ImageFormat::BGR);
		else if (image.channels() == 4)
			convert_image(image, output, format, ImageFormat::BGRA);

		cv::imwrite(filename, output);
	}

	void find_inner_boundary(const cv::Mat& mask, cv::Mat& output)
	{
		cv::Mat _mask = mask > 0;
		cv::erode(mask, output, cv::Mat());
		output = output ^ _mask;
	}

	void find_inner_boundary(const MATRIX_u8& mask, MATRIX_u8& output)
	{
		cv::Mat cv_mask = wrap_as_opencv_matrix(mask) > 0;
		cv::Mat cv_bds_mask;
		find_inner_boundary(cv_mask, cv_bds_mask);
		to_matrix(cv_bds_mask, output);
	}

	void binary_fill_holes(const cv::Mat& input, cv::Mat& output)
	{
		assert_throw(input.type() == CV_8UC1, "only binary image is supported");

		//append 1 pixel to all edges of the input, forming a canvas
		cv::Mat canvas(input.rows + 2, input.cols + 2, input.type(), cv::Scalar(0));
		cv::Rect sub_canvas_region(1, 1, input.cols, input.rows);
		input.copyTo(canvas(sub_canvas_region));

		//fill the canvas from top-left pixel
		cv::floodFill(canvas, cv::Point(0, 0), 255);
		canvas = ~canvas; //extract holes region
		canvas(sub_canvas_region) |= input; //paste back the input
		canvas(sub_canvas_region).copyTo(output);
	}

	void get_largest_connected_component(const cv::Mat& input, cv::Mat* output, int connectivity /*= 8*/)
	{
		if (!output)
			return;

		assert_throw(input.channels() == 1, "input must be a single-channel image");

		cv::Mat mask = input > 0;
		cv::Mat lbmap, stats, centroids;
		auto ncomp = cv::connectedComponentsWithStats(mask, lbmap, stats, centroids, connectivity);
		
		std::vector<int> areas(ncomp, 0);
		for (int i = 1; i < ncomp; i++)
		{
			areas[i] = stats.at<int>(i, cv::CC_STAT_AREA);
		}
		int idxmax = std::max_element(areas.begin(), areas.end()) - areas.begin();

		output->create(input.rows, input.cols, CV_8UC1);
		output->setTo(0);
		for (int i = 0; i < lbmap.total(); i++)
			if (lbmap.at<int>(i) == idxmax)
				output->at<uint8_t>(i) = 255;
	}

	void pyramid_up(cv::Mat& output, const cv::Mat& img, cv::Size upsize, boost::optional<double> _sigma /*= boost::none*/)
	{
		double diag_after = diagonal_length(upsize);
		double diag_before = diagonal_length({ img.cols, img.rows });

		assert_throw(diag_after > diag_before, "upsize is smaller than input size");

		double sigma = 0;
		if (_sigma) sigma = *_sigma;
		else {
			double scale = diag_after / diag_before;
			sigma = 2 * scale / 6.0; //see scipy pyramid_expand()
		}

		//int ksize = int(sigma * 2 + 0.5); //round(2*sigma)
		//if (ksize % 2 == 0)
		//	ksize = ksize + 1; //make it odd
		//ksize = ksize >= 3 ? ksize : 3;

		cv::resize(img, output, upsize, 0,0, cv::INTER_NEAREST);
		cv::GaussianBlur(output, output, cv::Size(0, 0), sigma);
	}

	void pyramid_down(cv::Mat& output, const cv::Mat& img, cv::Size downsize, boost::optional<double> _sigma /*= boost::none*/)
	{
		double diag_after = diagonal_length(downsize);
		double diag_before = diagonal_length({ img.cols, img.rows });

		assert_throw(diag_after < diag_before, "downsize is larger than than input size");

		double sigma = 0;
		if (_sigma) sigma = *_sigma;
		else {
			double scale = diag_after / diag_before;
			sigma = 2 * scale / 6.0; //see scipy pyramid_expand()
		}

		//int ksize = int(sigma * 2 + 0.5); //round(2*sigma)
		//if (ksize % 2 == 0)
		//	ksize = ksize + 1; //make it odd
		//ksize = ksize >= 3 ? ksize : 3;

		cv::GaussianBlur(img, output, cv::Size(0, 0), sigma);
		cv::resize(output, output, downsize, 0, 0, cv::INTER_NEAREST);
	}

	void multi_band_blending(cv::Mat& output, const cv::Mat& img_src, const cv::Mat& img_dst, const cv::Mat& weight_src, boost::optional<int> _n_band /*= boost::none*/, boost::optional<double> sigma /*= boost::none*/)
	{
		//defaults
		int n_band = _n_band ? *_n_band : std::numeric_limits<int>::max();

		//checks

		//image format and size of src and dst must be consistent
		assert_throw(is_type<uint8_t>(img_src) || is_type_floating_point(img_src), "unsupported image type");
		assert_throw(is_same_size(img_src, img_dst), "the src and dst images do not have the same size");
		assert_throw(is_same_format(img_src, img_dst), "the src and dst images do not have the same format");
		assert_throw(is_same_size(weight_src, img_src), "weight map and src do not have the same size");
		assert_throw(weight_src.channels() == 1, "weight map should have single channel");

		//convert images to float
		cv::Mat fimg_src, fimg_dst, fwmap_src;
		if (is_type_integer(img_src))
			img_src.convertTo(fimg_src, CV_FLOAT_TYPE, 1.0 / 255);
		else {
			assert(is_type_floating_point(img_src));
			fimg_src = img_src;
		}

		if (is_type_integer(img_dst))
			img_dst.convertTo(fimg_dst, CV_FLOAT_TYPE, 1.0 / 255);
		else {
			assert(is_type_floating_point(img_dst));
			fimg_dst = img_dst;
		}

		if (is_type_integer(weight_src))
			weight_src.convertTo(fwmap_src, fimg_src.type(), 1.0 / 255);
		else if (weight_src.depth() != fimg_src.depth()) //differ by 32bit and 64bit
			weight_src.convertTo(fwmap_src, fimg_src.type());
		else {
			assert(is_type_floating_point(weight_src));
			fwmap_src = weight_src;
		}
			
		// mixing the images level by level, from top to bottom
		cv::Mat src_thislv = fimg_src;
		cv::Mat dst_thislv = fimg_dst;
		cv::Mat wmap_thislv = fwmap_src;

		//laplacian pyramid of the blended image, from top to bottom
		std::vector<cv::Mat> mix_lp_pyramid;
		cv::Scalar ONE = make_scalar(1.0, img_src.channels());

		for (int i = 0; i < n_band; i++) {
			cv::Size cur_size(src_thislv.cols, src_thislv.rows);

			//check if the down sized image is too small
			cv::Size downsize(src_thislv.cols / 2, src_thislv.rows / 2);
			if (downsize.width <= 5 || downsize.height <= 5 || i >= n_band)
			{
				//this is the bottom level, mix the image directly
				cv::Mat wmap_with_ch;
				{
					std::vector<cv::Mat> chlist(fimg_src.channels(), wmap_thislv);
					cv::merge(chlist, wmap_with_ch);
				}
				cv::Mat blended = src_thislv.mul(wmap_with_ch) + dst_thislv.mul(ONE - wmap_with_ch);
				mix_lp_pyramid.push_back(blended);
				break;
			}

			//this is an intermediate level in laplacian pyramid, mix the difference
			cv::Mat src_down, dst_down;
			cv::Mat src_up, dst_up;
			cv::Mat src_diff, dst_diff, blend_diff;

			//downscale and then upscale the source, compute difference
			pyramid_down(src_down, src_thislv, downsize, sigma);
			pyramid_up(src_up, src_down, cur_size, sigma);
			src_diff = src_thislv - src_up;

			//do the same with dst
			pyramid_down(dst_down, dst_thislv, downsize, sigma);
			pyramid_up(dst_up, dst_down, cur_size, sigma);
			dst_diff = dst_thislv - dst_up;

			//mix the difference at this level
			cv::Mat wmap_with_ch;
			{
				std::vector<cv::Mat> chlist(fimg_src.channels(), wmap_thislv);
				cv::merge(chlist, wmap_with_ch);
			}
			blend_diff = src_diff.mul(wmap_with_ch) + dst_diff.mul(ONE - wmap_with_ch);
			mix_lp_pyramid.push_back(blend_diff);

			//get ready for next level
			src_thislv = src_down;
			dst_thislv = dst_down;
			pyramid_down(wmap_thislv, wmap_thislv, downsize, sigma);
		}

		//reconstruction
		cv::Mat recon = mix_lp_pyramid.back();
		for (auto it = mix_lp_pyramid.rbegin()+1; it != mix_lp_pyramid.rend(); ++it) {
			//ר�����
	/*		cv::Mat recon_out = recon.clone();
			clamp_values(recon_out, 0, 1);
			CvMatHelper hlpout(recon_out);
		
			auto _recon = hlpout.get_as_type(CV_8UC3);
			std::string fn_out_img = "D://3DCafe//�����걨����2018//ר��//2019//��ͼ//recon";
			
			fn_out_img  = fn_out_img + std::to_string(recon_out.rows)+ "x"+std::to_string(recon_out.cols) +".jpg";
		    imwrite(fn_out_img, _recon,ImageFormat::RGB);*/

			cv::Mat dif = *it;
			cv::Size upsize(dif.cols, dif.rows);
			pyramid_up(recon, recon, upsize, sigma);
			recon += dif;
		}

		//clip the values
		clamp_values(recon, 0, 1);

		//ר�����
		/*CvMatHelper hlpout(recon);

		auto _recon = hlpout.get_as_type(CV_8UC3);
		std::string fn_out_img = "D://3DCafe//�����걨����2018//ר��//2019//��ͼ//recon";

		fn_out_img = fn_out_img + std::to_string(_recon.rows) + "x" + std::to_string(_recon.cols) + ".jpg";
		imwrite(fn_out_img, _recon, ImageFormat::RGB);*/

		//convert back to original type
		if (is_type<uint8_t>(img_src)) {
			recon.convertTo(output, img_src.type(), 255.0);
		}
		else {
			recon.convertTo(output, img_src.type());
		}
	}
}


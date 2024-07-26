#pragma once

#include "def_magnum.h"
#include "util_magnum_eigen.h"
#include <limits>

namespace _NS_UTILITY {
	namespace MagnumProc {
		class SceneNode : public Object3D {
		public:
			using Object3D::Object;

			/**
			* \brief set the position of this node
			*
			* \param pos the position
			* \param cs the coordinate system where position is specified
			*/
			void set_position(const Magnum::Vector3& pos, CoordinateSystem cs = CoordinateSystem::PARENT);

			/** \brief get the position of this node, relative to a specified coordinate system */
			Magnum::Vector3 get_position(CoordinateSystem cs = CoordinateSystem::PARENT) const;

			/** \brief get the 3x3 rotation matrix, relative to a specified coordinate system */
			void get_orientation(Magnum::Matrix3& output, CoordinateSystem cs = CoordinateSystem::PARENT) const;

			/** \brief get the 4x4 rotation matrix, relative to a specified coordinate system */
			void get_orientation(Magnum::Matrix4& output, CoordinateSystem cs = CoordinateSystem::PARENT) const;

			/** \brief set the orientation given in a speicifed coordinate system */
			void set_orientation(const Magnum::Matrix3& rotmat, CoordinateSystem cs = CoordinateSystem::PARENT);

			/** \brief set the orientation given in a speicifed coordinate system */
			void set_orientation(const Magnum::Matrix4& rotmat, CoordinateSystem cs = CoordinateSystem::PARENT);

			/** \brief orient the node so that it looks at a point and its Y+ axis is 
			aligned with the up_vector */
			void set_look_at(const Magnum::Vector3& pos,
				const Magnum::Vector3& up_vector = Magnum::Vector3{ 0.f, 1.f, 0.f },
				CoordinateSystem cs = CoordinateSystem::PARENT);

			/** \brief given a point in a coordinate system, find its coordinate in another coordinate system  */
			Magnum::Vector3 convert_position(
				const Magnum::Vector3& pos, 
				CoordinateSystem cs_from, 
				CoordinateSystem cs_to);

			/** \brief represent the rotation in one coordinate system to another */
			Magnum::Matrix3 convert_orientation(const Magnum::Matrix3& rotmat, 
				CoordinateSystem cs_from, 
				CoordinateSystem cs_to);

			/** \brief represent the vector in one coordinate system to another */
			Magnum::Vector3 convert_vector(const Magnum::Vector3 &vec, CoordinateSystem cs_from, CoordinateSystem cs_to);

		private:
			Magnum::Matrix4 get_coordinate_frame_relative_to_world(CoordinateSystem cs);
		};

		// ================== implementation ======================
		inline void SceneNode::set_position(const Magnum::Vector3& pos, CoordinateSystem cs)
		{
			if (cs == CoordinateSystem::PARENT)
			{
				auto tmat = this->transformationMatrix();

				//the magnum matrix is column-major
				//set the position into the matrix
				auto _tmat = Eigen::Map<Eigen::Matrix<Magnum::Float, 4, 4>>(tmat.data());
				_tmat(0, 3) = pos.x();
				_tmat(1, 3) = pos.y();
				_tmat(2, 3) = pos.z();

				this->setTransformation(tmat);
			}
			else if (cs == CoordinateSystem::SELF)
			{
				this->translateLocal(pos);
			}
			else if (cs == CoordinateSystem::WORLD)
			{
				//convert the position to parent and set it again
				auto _pos = convert_position(pos, CoordinateSystem::WORLD, CoordinateSystem::PARENT);
				set_position(_pos, CoordinateSystem::PARENT);
			}
		}

		inline Magnum::Vector3 SceneNode::get_position(CoordinateSystem cs) const
		{
			if (cs == CoordinateSystem::PARENT)
				return this->transformationMatrix().translation();
			else if (cs == CoordinateSystem::SELF)
				return { 0.f,0.f,0.f };
			else if (cs == CoordinateSystem::WORLD)
				return this->absoluteTransformation().translation();
			else
				return Magnum::Vector3(std::numeric_limits<Magnum::Float>::max()); //should not reach here
		}

		inline void SceneNode::get_orientation(Magnum::Matrix3& output, CoordinateSystem cs /*= CoordinateSystem::PARENT*/) const
		{
			if (cs == CoordinateSystem::PARENT)
				output = this->transformationMatrix().rotation();
			else if (cs == CoordinateSystem::SELF)
				output = Magnum::Matrix3{};
			else if (cs == CoordinateSystem::WORLD)
				output = this->absoluteTransformation().rotation();
		}

		inline void SceneNode::get_orientation(Magnum::Matrix4& output, CoordinateSystem cs /*= CoordinateSystem::PARENT*/) const
		{
			Magnum::Matrix3 rotmat;
			if (cs == CoordinateSystem::PARENT)
				rotmat = this->transformationMatrix().rotation();
			else if (cs == CoordinateSystem::SELF)
				rotmat = Magnum::Matrix3{};
			else if (cs == CoordinateSystem::WORLD)
				rotmat = this->absoluteTransformation().rotation();

			output = Magnum::Matrix4{};
			auto _rotmat = Eigen::Map<Eigen::Matrix<Magnum::Float, 3, 3>>(rotmat.data());
			auto _output = Eigen::Map<Eigen::Matrix<Magnum::Float, 4, 4>>(output.data());

			_output.block(0, 0, 3, 3) = _rotmat;
		}

		inline void SceneNode::set_orientation(const Magnum::Matrix3& rotmat, CoordinateSystem cs /*= CoordinateSystem::PARENT*/)
		{
			auto func_set_rotation = [](Magnum::Matrix4& tmat, const Magnum::Matrix3& rotmat) {
				auto _tmat = to_eigen_map_colmajor(tmat);

				//if tmat has scaling component, we have to preserve it
				Magnum::Vector3 scalevec = tmat.scaling();
				auto scalemat = Magnum::Matrix3::fromDiagonal(scalevec);
				auto rot_scale = scalemat * rotmat;				

				auto _rot_scale = to_eigen_map_colmajor(rot_scale);
				_tmat.block(0, 0, 3, 3) = _rot_scale;
			};

			if (cs == CoordinateSystem::PARENT)
			{
				auto tmat = this->transformationMatrix();
				func_set_rotation(tmat, rotmat);
				this->setTransformation(tmat);
			}
			else if (cs == CoordinateSystem::SELF) {
				auto tmat = this->transformationMatrix();
				Magnum::Matrix3 new_rotmat = rotmat * tmat.rotation();
				func_set_rotation(tmat, new_rotmat);
				this->setTransformation(tmat);
			}
			else if (cs == CoordinateSystem::WORLD) {
				auto _rotmat = convert_orientation(rotmat, CoordinateSystem::WORLD, CoordinateSystem::PARENT);
				set_orientation(_rotmat, CoordinateSystem::PARENT);
			}
		}

		inline void SceneNode::set_orientation(const Magnum::Matrix4& rotmat, CoordinateSystem cs /*= CoordinateSystem::PARENT*/)
		{
			set_orientation(rotmat.rotation(), cs);
		}

		inline void SceneNode::set_look_at(
			const Magnum::Vector3& pos,const Magnum::Vector3& up_vector,
			CoordinateSystem cs)
		{
			auto _pos = convert_position(pos, cs, CoordinateSystem::PARENT);
			auto _up_vector = convert_vector(up_vector, cs, CoordinateSystem::PARENT);
			auto tmat = Magnum::Matrix4::lookAt(get_position(CoordinateSystem::PARENT), _pos, _up_vector);
			set_orientation(tmat.rotation());
		}

		inline Magnum::Vector3 SceneNode::convert_position(
			const Magnum::Vector3& pos, CoordinateSystem cs_from, CoordinateSystem cs_to)
		{
			if (cs_from == cs_to) return pos;
			auto frame_from = get_coordinate_frame_relative_to_world(cs_from);
			auto frame_to = get_coordinate_frame_relative_to_world(cs_to);
			return frame_to.inverted().transformPoint(frame_from.transformPoint(pos));
		}

		inline Magnum::Matrix3 SceneNode::convert_orientation(
			const Magnum::Matrix3& rotmat, CoordinateSystem cs_from, CoordinateSystem cs_to)
		{
			if (cs_from == cs_to) return rotmat;
			auto frame_from = get_coordinate_frame_relative_to_world(cs_from);
			auto frame_to = get_coordinate_frame_relative_to_world(cs_to);
			return frame_to.rotation().transposed() * frame_from.rotation() * rotmat;
		}

		inline Magnum::Vector3 SceneNode::convert_vector(
			const Magnum::Vector3 &vec, CoordinateSystem cs_from, CoordinateSystem cs_to)
		{
			if (cs_from == cs_to) return vec;
			auto frame_from = get_coordinate_frame_relative_to_world(cs_from);
			auto frame_to = get_coordinate_frame_relative_to_world(cs_to);
			return frame_to.inverted().transformVector(frame_from.transformVector(vec));
		}

		inline Magnum::Matrix4 SceneNode::get_coordinate_frame_relative_to_world(CoordinateSystem cs)
		{
			switch (cs) {
			case CoordinateSystem::PARENT:
				if (this->parent()) return this->parent()->absoluteTransformation();
				else return {};
			case CoordinateSystem::SELF:
				return this->absoluteTransformation();
			case CoordinateSystem::WORLD:
				return {}; //world relative to world, it is identity
			default:
				return Magnum::Matrix4{std::numeric_limits<Magnum::Float>::max()}; //should not reach here
			}
		}

	}
}

#include <igcclib/visualization/soft_renderer/softlit/Master.h>
#include <igcclib/visualization/soft_renderer/softlit/Primitive.h>
#include <igcclib/visualization/soft_renderer/softlit/Texture.h>

namespace softlit
{
	Primitive::Primitive(const PrimitiveSetup& setup)
		: m_setup(setup)
	{
	}

	Primitive::~Primitive()
	{
		m_vertexBuffer.clear();
		m_indexBuffer.clear();

		//MODIFY: you do not own the texture
		//for (Texture* t : m_tbo)
		//	SAFE_DELETE(t);
		m_tbo.clear();
	}
}
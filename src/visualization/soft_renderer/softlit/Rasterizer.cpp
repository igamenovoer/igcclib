#include <igcclib/visualization/soft_renderer/softlit/Master.h>
#include <igcclib/visualization/soft_renderer/softlit/Rasterizer.h>
#include <igcclib/visualization/soft_renderer/softlit/Shaders.h>

using std::vector;

namespace softlit
{
	Rasterizer::Rasterizer(const RasterizerSetup& setup)
		: m_setup(setup)
	{
		//m_frameBuffer.resize(m_setup.viewport.width * m_setup.viewport.height);
		m_frameBuffer.init(m_setup.viewport.width, m_setup.viewport.height, ALL_OUTPUT_CHANNELS);
        m_depthBuffer.resize(m_setup.viewport.width * m_setup.viewport.height);
    }

    Rasterizer::~Rasterizer()
    {
        m_frameBuffer.clear_color();
        m_depthBuffer.clear();
    }

    /*
            - v1
          -	 -	 -  -
        vo	-	-	- v2
    */
    inline float Rasterizer::PixelCoverage(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) const
    {
        const int winding = (m_setup.vertexWinding == VertexWinding::COUNTER_CLOCKWISE) ? 1 : -1;
        const float x = (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);

        return winding * x;
    }

    void Rasterizer::SetupTriangle(Primitive* prim, const int64_t idx, glm::vec3& v0, glm::vec3& v1, glm::vec3& v2) const
    {
        const VertexBuffer& vbo = prim->getVertexBuffer();
        const IndexBuffer& ibo = prim->getIndexBuffer();

        v0 = vbo[ibo[idx * 3]];
        v1 = vbo[ibo[idx * 3 + 1]];
        v2 = vbo[ibo[idx * 3 + 2]];
    }

	void Rasterizer::SetupTriangle(Primitive* prim, const int64_t idx, 
		glm::vec3& v0, glm::vec3& v1, glm::vec3& v2, 
		int64_t& idx0, int64_t& idx1, int64_t& idx2) const
	{
		const VertexBuffer& vbo = prim->getVertexBuffer();
		const IndexBuffer& ibo = prim->getIndexBuffer();

		idx0 = idx * 3;
		idx1 = idx0 + 1;
		idx2 = idx0 + 2;

		v0 = vbo[ibo[idx0]];
		v1 = vbo[ibo[idx1]];
		v2 = vbo[ibo[idx2]];

		bool need_flip_cw = m_setup.vertexWinding == VertexWinding::CLOCKWISE
			&& glm::cross(v1 - v0, v2 - v0).z > 0;
		bool need_flip_ccw = m_setup.vertexWinding == VertexWinding::COUNTER_CLOCKWISE
			&& glm::cross(v1 - v0, v2 - v0).z < 0;
		if (need_flip_ccw || need_flip_cw)
		{
			//swap two vertices
			idx0 = idx * 3;
			idx1 = idx0 + 2;
			idx2 = idx0 + 1;

			v0 = vbo[ibo[idx0]];
			v1 = vbo[ibo[idx1]];
			v2 = vbo[ibo[idx2]];
		}
	}

	bool softlit::Rasterizer::Clip2D(const glm::vec2 & v0, const glm::vec2 & v1, const glm::vec2 & v2, Viewport& vp) const
    {
		using std::min;
		using std::max;

        float xmin = min(v0.x, min(v1.x, v2.x));
        float xmax = max(v0.x, max(v1.x, v2.x));
        float ymin = min(v0.y, min(v1.y, v2.y));
        float ymax = max(v0.y, max(v1.y, v2.y));

        // Early-out when viewport bounds exceeded
        if (xmin + 1 > m_setup.viewport.width || xmax < 0 || ymin + 1 > m_setup.viewport.height || ymax < 0) return false;

        vp.x = max<uint32_t>(0, (int32_t)xmin);
        vp.width = min<uint32_t>(m_setup.viewport.width - 1, (int32_t)xmax);
        vp.y = max<uint32_t>(0, (int32_t)ymin);
        vp.height = min<uint32_t>(m_setup.viewport.height - 1, (int32_t)ymax);

        return true;
    }

    void Rasterizer::InterpolateAttributes(const float u, const float v, const float w, const Vertex_OUT& out0, const Vertex_OUT& out1, const Vertex_OUT& out2, Vertex_OUT& attribs) const
    {
        if (!out0.attrib_vec4.empty())
        {
            DBG_ASSERT(!out1.attrib_vec4.empty());
            DBG_ASSERT(!out2.attrib_vec4.empty());

            DBG_ASSERT(out0.attrib_vec4.size() == out2.attrib_vec4.size());
            DBG_ASSERT(out1.attrib_vec4.size() == out2.attrib_vec4.size());

            for (int i = 0; i < out0.attrib_vec4.size(); i++)
            {
                const glm::vec4& attr0 = out0.attrib_vec4[i];
                const glm::vec4& attr1 = out1.attrib_vec4[i];
                const glm::vec4& attr2 = out2.attrib_vec4[i];

                glm::vec4 attrib = u * attr0 + v * attr1 + w * attr2;

                attribs.PushVertexAttribute(attrib);
            }
        }

        if (!out0.attrib_vec3.empty())
        {
            DBG_ASSERT(!out1.attrib_vec3.empty());
            DBG_ASSERT(!out2.attrib_vec3.empty());

            DBG_ASSERT(out0.attrib_vec3.size() == out2.attrib_vec3.size());
            DBG_ASSERT(out1.attrib_vec3.size() == out2.attrib_vec3.size());

            for (int i = 0; i < out0.attrib_vec3.size(); i++)
            {
                const glm::vec3& attr0 = out0.attrib_vec3[i];
                const glm::vec3& attr1 = out1.attrib_vec3[i];
                const glm::vec3& attr2 = out2.attrib_vec3[i];

                glm::vec3 attrib = u * attr0 + v * attr1 + w * attr2;

                attribs.PushVertexAttribute(attrib);
            }
        }

        if (!out0.attrib_vec2.empty())
        {
            DBG_ASSERT(!out1.attrib_vec2.empty());
            DBG_ASSERT(!out2.attrib_vec2.empty());

            DBG_ASSERT(out0.attrib_vec2.size() == out2.attrib_vec2.size());
            DBG_ASSERT(out1.attrib_vec2.size() == out2.attrib_vec2.size());

            for (int i = 0; i < out0.attrib_vec2.size(); i++)
            {
                const glm::vec2& attr0 = out0.attrib_vec2[i];
                const glm::vec2& attr1 = out1.attrib_vec2[i];
                const glm::vec2& attr2 = out2.attrib_vec2[i];

                glm::vec2 attrib = u * attr0 + v * attr1 + w * attr2;

                attribs.PushVertexAttribute(attrib);
            }
        }
    }

    void Rasterizer::FetchVertexAttributes(const VertexAttributes& attribs, const int64_t idx, Vertex_IN& in0, Vertex_IN& in1, Vertex_IN& in2) const
    {
        // Fetch attributes of type vec4
        if (!attribs.attrib_vec4.empty()) // At least one vec4 attribute buffer created
        {
            for (int i = 0; i < attribs.attrib_vec4.size(); i++)
            {
                in0.PushVertexAttribute(attribs.attrib_vec4[i][idx * 3]);
                in1.PushVertexAttribute(attribs.attrib_vec4[i][idx * 3 + 1]);
                in2.PushVertexAttribute(attribs.attrib_vec4[i][idx * 3 + 2]);
            }
        }

        // Fetch attributes of type vec3
        if (!attribs.attrib_vec3.empty()) // At least one vec4 attribute buffer created
        {
            for (int i = 0; i < attribs.attrib_vec3.size(); i++)
            {
                in0.PushVertexAttribute(attribs.attrib_vec3[i][idx * 3]);
                in1.PushVertexAttribute(attribs.attrib_vec3[i][idx * 3 + 1]);
                in2.PushVertexAttribute(attribs.attrib_vec3[i][idx * 3 + 2]);
            }
        }

        // Fetch attributes of type vec2
        if (!attribs.attrib_vec2.empty()) // At least one vec4 attribute buffer created
        {
            for (int i = 0; i < attribs.attrib_vec2.size(); i++)
            {
                in0.PushVertexAttribute(attribs.attrib_vec2[i][idx * 3]);
                in1.PushVertexAttribute(attribs.attrib_vec2[i][idx * 3 + 1]);
                in2.PushVertexAttribute(attribs.attrib_vec2[i][idx * 3 + 2]);
            }
        }
    }

	void Rasterizer::FetchVertexAttributes(
		const VertexAttributes& attribs, 
		const int64_t idx0, const int64_t idx1, const int64_t idx2, 
		Vertex_IN& in0, Vertex_IN& in1, Vertex_IN& in2) const
	{
		// Fetch attributes of type vec4
		if (!attribs.attrib_vec4.empty()) // At least one vec4 attribute buffer created
		{
			for (int i = 0; i < attribs.attrib_vec4.size(); i++)
			{
				in0.PushVertexAttribute(attribs.attrib_vec4[i][idx0]);
				in1.PushVertexAttribute(attribs.attrib_vec4[i][idx1]);
				in2.PushVertexAttribute(attribs.attrib_vec4[i][idx2]);
			}
		}

		// Fetch attributes of type vec3
		if (!attribs.attrib_vec3.empty()) // At least one vec4 attribute buffer created
		{
			for (int i = 0; i < attribs.attrib_vec3.size(); i++)
			{
				in0.PushVertexAttribute(attribs.attrib_vec3[i][idx0]);
				in1.PushVertexAttribute(attribs.attrib_vec3[i][idx1]);
				in2.PushVertexAttribute(attribs.attrib_vec3[i][idx2]);
			}
		}

		// Fetch attributes of type vec2
		if (!attribs.attrib_vec2.empty()) // At least one vec4 attribute buffer created
		{
			for (int i = 0; i < attribs.attrib_vec2.size(); i++)
			{
				in0.PushVertexAttribute(attribs.attrib_vec2[i][idx0]);
				in1.PushVertexAttribute(attribs.attrib_vec2[i][idx1]);
				in2.PushVertexAttribute(attribs.attrib_vec2[i][idx2]);
			}
		}
	}

	void Rasterizer::Draw(Primitive* prim)
    {
        // Only raster primitive is triangle
        const int64_t numTris = prim->getIndexBuffer().size() / 3;
        DBG_ASSERT((prim->getIndexBuffer().size() % 3) == 0);

        // Re-use VS attributes per primitive
        Vertex_IN in0, in1, in2;
        Vertex_OUT out0, out1, out2;

        // Re-use FS attributes per primitive
        Vertex_OUT FS_attribs;
		const auto& fs_output_mapping = prim->getFS()->get_output_mapping();
		bool output_barycentric = m_frameBuffer.has_channel(ShaderOutputChannel::BARYCENTRIC);

#pragma omp parallel for private(in0, in1, in2, out0, out1, out2, FS_attribs)
        for (int64_t i = 0; i < numTris; i++)
        {
            glm::vec3 v0, v1, v2;
			//int64_t i0, i1, i2;
			SetupTriangle(prim, i, v0, v1, v2);
            //SetupTriangle(prim, i, v0, v1, v2, i0, i1, i2);

            const vertex_shader VS = prim->getVS();
            DBG_ASSERT(VS && "invalid vertex_shader!");

            //UniformBuffer ubo = prim->getUBO();
			auto ubo = prim->getUBO_ex();

            in0.ResetData(); in1.ResetData(); in2.ResetData();
            out0.ResetData(); out1.ResetData(); out2.ResetData();
            FetchVertexAttributes(prim->getVertexAttributes(), i, in0, in1, in2);
			//FetchVertexAttributes(prim->getVertexAttributes(), i0, i1, i2, in0, in1, in2);

            // Execute VS for each vertex
			glm::vec4 v0Clip = VS->do_shading(v0, ubo, &in0, &out0);
			glm::vec4 v1Clip = VS->do_shading(v1, ubo, &in1, &out1);
			glm::vec4 v2Clip = VS->do_shading(v2, ubo, &in2, &out2);
            //vec4 v0Clip = VS(v0, ubo, &in0, &out0);
            //vec4 v1Clip = VS(v1, ubo, &in1, &out1);
            //vec4 v2Clip = VS(v2, ubo, &in2, &out2);

            const float oneOverW0 = 1.f / v0Clip.w;
            const float oneOverW1 = 1.f / v1Clip.w;
            const float oneOverW2 = 1.f / v2Clip.w;

            // Perspective-divide and convert to NDC
            glm::vec3 v0NDC = v0Clip * oneOverW0;
            glm::vec3 v1NDC = v1Clip * oneOverW1;
            glm::vec3 v2NDC = v2Clip * oneOverW2;

            // Now to frame buffer-coordinates
            glm::vec2 v0Raster = { (v0NDC.x + 1) / 2 * m_setup.viewport.width, (1 - v0NDC.y) / 2 * m_setup.viewport.height };
            glm::vec2 v1Raster = { (v1NDC.x + 1) / 2 * m_setup.viewport.width, (1 - v1NDC.y) / 2 * m_setup.viewport.height };
            glm::vec2 v2Raster = { (v2NDC.x + 1) / 2 * m_setup.viewport.width, (1 - v2NDC.y) / 2 * m_setup.viewport.height };

            float triCoverage = PixelCoverage(v0Raster, v1Raster, v2Raster);
			bool is_backface = (triCoverage > 0 && m_setup.vertexWinding == VertexWinding::CLOCKWISE) ||
				(triCoverage < 0 && m_setup.vertexWinding == VertexWinding::COUNTER_CLOCKWISE);

			if (is_backface)
			{
				auto cullmode = prim->getPrimitiveSetup().cullMode;
				if(cullmode == CullMode::CULL_BACK)
					continue;
				else {
					//swap vertices
					std::swap(v1, v2);
					std::swap(v1NDC, v2NDC);
					std::swap(v1Raster, v2Raster);
					std::swap(in1, in2);
					std::swap(out1, out2);

					//update coverage again
					triCoverage = PixelCoverage(v0Raster, v1Raster, v2Raster);
				}
			}

			//triCoverage = PixelCoverage(v0Raster, v1Raster, v2Raster);
   //         if ((prim->getPrimitiveSetup().cullMode == CullMode::CULL_BACK) &&
   //             (triCoverage > 0 && m_setup.vertexWinding == VertexWinding::CLOCKWISE) ||
   //             (triCoverage < 0 && m_setup.vertexWinding == VertexWinding::COUNTER_CLOCKWISE)) continue;

            Viewport vp;
            if (!Clip2D(v0Raster, v1Raster, v2Raster, vp)) continue;

            // Store edges
            const glm::vec2 edge0 = v2 - v1;
            const glm::vec2 edge1 = v0 - v2;
            const glm::vec2 edge2 = v1 - v0;

            // Pre-compute a flag for each edge to check for shared vertices and only include bottom/left edges
            const bool t0 = (edge0.x != 0) ? (edge0.x > 0) : (edge0.y > 0);
            const bool t1 = (edge1.x != 0) ? (edge1.x > 0) : (edge1.y > 0);
            const bool t2 = (edge2.x != 0) ? (edge2.x > 0) : (edge2.y > 0);

            // Triangle traversal
			std::vector<glm::vec4> fragdata(prim->getFS()->num_output_channel());
            for (uint32_t y = vp.y; y <= vp.height; y++)
            {
                for (uint32_t x = vp.x; x <= vp.width; x++)
                {
                    glm::vec2 sample = { x + 0.5f, y + 0.5f };

                    // Evaluate edge functions on sample
                    float e0 = PixelCoverage(v1Raster, v2Raster, sample);
                    float e1 = PixelCoverage(v2Raster, v0Raster, sample);
                    float e2 = PixelCoverage(v0Raster, v1Raster, sample);

                    bool included = true;
                    included &= (e0 == 0) ? t0 : (e0 > 0);
                    included &= (e1 == 0) ? t1 : (e1 > 0);
                    included &= (e2 == 0) ? t2 : (e2 > 0);

                    if (included)
                    {
                        e0 /= triCoverage;
                        e1 /= triCoverage;
                        e2 = 1.f - e0 - e1;

                        // Interpolate depth
                        float z = (e0 * v0NDC.z) + (e1 * v1NDC.z) + (e2 * v2NDC.z);

                        if (z < m_depthBuffer[y * m_setup.viewport.width + x]) // Depth test; execute FS if passed & update z-buffer
                        {
							m_depthBuffer[y * m_setup.viewport.width + x] = z;
                            FS_attribs.ResetData(); // Reset attribs pre-FS for each fragment

                            const float f0 = e0 * oneOverW0;
                            const float f1 = e1 * oneOverW1;
                            const float f2 = e2 * oneOverW2;

                            // Calc barycentric coordinates for perspectively-correct interpolation
                            const float u = f0 / (f0 + f1 + f2);
                            const float v = f1 / (f0 + f1 + f2);
                            const float w = 1.f - u - v;

                            InterpolateAttributes(u, v, w, out0, out1, out2, FS_attribs);
                            const fragment_shader FS = prim->getFS();
                            //const vec4 final_fragment = FS(ubo, &FS_attribs, prim->getTBO());
							FS->do_shading(fragdata.data(),VS.get(), ubo, &FS_attribs, prim->getTBO());

                            //m_frameBuffer[y * m_setup.viewport.width + x] = final_fragment;
							for (auto& it : fs_output_mapping)
							{
								m_frameBuffer.set_color(it.first, y, x, fragdata[it.second]);
							}

							glm::vec4 bcvalue((float)prim->get_id(), (float)i, u, v);
							if(output_barycentric)
								m_frameBuffer.set_color(ShaderOutputChannel::BARYCENTRIC, y, x, bcvalue);
							//m_frameBuffer.set_color(ShaderOutputChannel::FINAL_COLOR, y, x, fragdata[0]);
                        }
                    }
                }
            }
        }
    }
}
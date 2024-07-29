#pragma once
#include "io_fbx.h"
#include "fbxsdk.h"

namespace _NS_UTILITY {

	void read_mesh_list_from_fbx_file(const std::string& filename,std::map<std::string,TriangularMesh>& out_mesh_list)
	{
			// Initialize the SDK manager.This object handles memory management.
		fbxsdk::FbxManager*  lSdkManager = fbxsdk::FbxManager::Create();

		// Create the IO settings object.
		fbxsdk::FbxIOSettings *ios = fbxsdk::FbxIOSettings::Create(lSdkManager, IOSROOT);
		lSdkManager->SetIOSettings(ios);

		// Create an importer using the SDK manager.
		fbxsdk::FbxImporter* lImporter = fbxsdk::FbxImporter::Create(lSdkManager, "");

		// Use the first argument as the filename for the importer.
		if (!lImporter->Initialize(filename.c_str(), -1, lSdkManager->GetIOSettings())) {

			printf("Error returned: %s\n\n", lImporter->GetStatus().GetErrorString());
			assert_throw(false, "Call to FbxImporter::Initialize() failed.\n");
		}
		// Create a new scene so that it can be populated by the imported file.
		fbxsdk::FbxScene* lScene = fbxsdk::FbxScene::Create(lSdkManager, "myScene");

		// Import the contents of the file into the scene.
		lImporter->Import(lScene);

		// The file is imported, so get rid of the importer.
		lImporter->Destroy();


		auto root_node = lScene->GetRootNode();
		for (int i =0;i< lScene->GetNodeCount();i++)
		{
			auto node = root_node->GetChild(i);
			if (node)
			{
				TriangularMesh mesh;
				read_mesh_from_fbx_node(node, mesh);

				out_mesh_list[mesh.get_name()] = mesh;
			}
			
		}		

		lSdkManager->Destroy();
	}

	fMATRIX_4 get_local_transformation(const  fbxsdk::FbxNode* node)
	{
		const FbxVector4 lT = node->GetGeometricTranslation(FbxNode::eSourcePivot);
		const FbxVector4 lR = node->GetGeometricRotation(FbxNode::eSourcePivot);//angles in degree
		const FbxVector4 lS = node->GetGeometricScaling(FbxNode::eSourcePivot);

		fVECTOR_3 translate(lT.mData[0], lT.mData[1], lT.mData[2]);
		fVECTOR_3 rotate(lR.mData[0], lR.mData[1], lR.mData[2]);
		fVECTOR_3 scale(lS.mData[0], lS.mData[1], lS.mData[2]);
		rotate = rotate.array() * MathConstant::Deg2Rad;

		return transformation_matrix(translate, rotate, scale);
	}

	fMATRIX_4 get_global_transformation(fbxsdk::FbxNode*  node)
	{
		auto tm = node->EvaluateGlobalTransform();

		//const FbxVector4 lT = node->LclTranslation.Get();
		//const FbxVector4 lR = node->LclRotation.Get();//angles in degree
		//const FbxVector4 lS = node->LclScaling.Get();
		const FbxVector4 lT = tm.GetT();
		const FbxVector4 lR = tm.GetR();//angles in degree
		const FbxVector4 lS = tm.GetS();

		fVECTOR_3 translate(lT.mData[0], lT.mData[1], lT.mData[2]);
		fVECTOR_3 rotate(lR.mData[0], lR.mData[1], lR.mData[2]);
		fVECTOR_3 scale(lS.mData[0], lS.mData[1], lS.mData[2]);
		rotate = rotate.array() * MathConstant::Deg2Rad;


		return transformation_matrix(translate, rotate, scale);
	}

	void LoadUVInofrmation(fbxsdk::FbxMesh* pMesh, std::vector<double>& texcoord_vec, std::vector<int>& uv_faces_vec)
	{
		uv_faces_vec.clear();
		//get all UV set names
		FbxStringList lUVSetNameList;
		pMesh->GetUVSetNames(lUVSetNameList);

		//iterating over all uv sets
		for (int lUVSetIndex = 0; lUVSetIndex < lUVSetNameList.GetCount(); lUVSetIndex++)
		{
			//get lUVSetIndex-th uv set
			const char* lUVSetName = lUVSetNameList.GetStringAt(lUVSetIndex);
			const FbxGeometryElementUV* lUVElement = pMesh->GetElementUV(lUVSetName);

			if (!lUVElement)
				continue;

			// only support mapping mode eByPolygonVertex and eByControlPoint
			if (lUVElement->GetMappingMode() != FbxGeometryElement::eByPolygonVertex &&
				lUVElement->GetMappingMode() != FbxGeometryElement::eByControlPoint)
				return;

			//index array, where holds the index referenced to the uv data
			const bool lUseIndex = lUVElement->GetReferenceMode() != FbxGeometryElement::eDirect;
			const int lIndexCount = (lUseIndex) ? lUVElement->GetIndexArray().GetCount() : 0;

			//iterating through the data by polygon
			const int lPolyCount = pMesh->GetPolygonCount();

			if (lUVElement->GetMappingMode() == FbxGeometryElement::eByControlPoint)
			{
				assert_throw(false, "FbxGeometryElement::eByControlPoin dos not implement");
				for (int lPolyIndex = 0; lPolyIndex < lPolyCount; ++lPolyIndex)
				{
					// build the max index array that we need to pass into MakePoly
					const int lPolySize = pMesh->GetPolygonSize(lPolyIndex);
					for (int lVertIndex = 0; lVertIndex < lPolySize; ++lVertIndex)
					{
						FbxVector2 lUVValue;

						//get the index of the current vertex in control points array
						int lPolyVertIndex = pMesh->GetPolygonVertex(lPolyIndex, lVertIndex);

						//the UV index depends on the reference mode
						int lUVIndex = lUseIndex ? lUVElement->GetIndexArray().GetAt(lPolyVertIndex) : lPolyVertIndex;

						lUVValue = lUVElement->GetDirectArray().GetAt(lUVIndex);
						texcoord_vec[lUVIndex * 2] = lUVValue.mData[0];
						texcoord_vec[lUVIndex * 2 + 1] = lUVValue.mData[1];

						uv_faces_vec.push_back(lUVIndex);
						//User TODO:
						//Print out the value of UV(lUVValue) or log it to a file
					}
				}
			}
			else if (lUVElement->GetMappingMode() == FbxGeometryElement::eByPolygonVertex)
			{
				int lPolyIndexCounter = 0;
				for (int lPolyIndex = 0; lPolyIndex < lPolyCount; ++lPolyIndex)
				{
					// build the max index array that we need to pass into MakePoly
					const int lPolySize = pMesh->GetPolygonSize(lPolyIndex);
					for (int lVertIndex = 0; lVertIndex < lPolySize; ++lVertIndex)
					{
						if (lPolyIndexCounter < lIndexCount)
						{
							FbxVector2 lUVValue;

							//the UV index depends on the reference mode
							int lUVIndex = lUseIndex ? lUVElement->GetIndexArray().GetAt(lPolyIndexCounter) : lPolyIndexCounter;

							lUVValue = lUVElement->GetDirectArray().GetAt(lUVIndex);
							texcoord_vec[lUVIndex * 2] = lUVValue.mData[0];
							texcoord_vec[lUVIndex * 2 + 1] = lUVValue.mData[1];

							uv_faces_vec.push_back(lUVIndex);

							//std::cout << "lPolyIndex:" << lPolyIndex << std::endl;
							//std::cout << "lUVIndex:" << lUVIndex<< std::endl;
							//std::cout << "lUVValue:(" << lUVValue.mData[0] << ","<< lUVValue.mData[1]<<")" <<std::endl;

							//User TODO:
							//Print out the value of UV(lUVValue) or log it to a file

							lPolyIndexCounter++;
						}
					}
				}
			}
		}
	}


	void read_mesh_from_fbx_node(fbxsdk::FbxNode* node, TriangularMesh& out_mesh)
	{
		auto mesh = node->GetMesh();

		const char* nodeName = node->GetName();
		out_mesh.set_name(nodeName);

		//get local transformation
		auto local_tmat = get_local_transformation(node);

		//read vertices
		fMATRIX v_org;
		auto v_data = mesh->GetControlPoints();
		auto v_num = mesh->GetControlPointsCount();

		std::vector<double> vertices_vec;
		for (int i = 0; i < v_num; i++)
		{
			vertices_vec.push_back(v_data[i].mData[0]);
			vertices_vec.push_back(v_data[i].mData[1]);
			vertices_vec.push_back(v_data[i].mData[2]);
		}
		v_org = Eigen::Map<fMATRIX>(vertices_vec.data(), v_num, 3);
		auto v = transform_points(v_org, local_tmat);
		//std::cout << "local_tmat" << local_tmat << std::endl;
		//read faces
		iMATRIX f;
		auto f_num = mesh->GetPolygonCount();
		{
			auto f_data = mesh->GetPolygonVertices();

			std::vector<int> faces_vec;

			for (int idx = 0; idx < f_num; idx++)
			{
				auto i = f_data[idx * 3];
				auto j = f_data[idx * 3 + 1];
				auto k = f_data[idx * 3 + 2];
				faces_vec.push_back(i);
				faces_vec.push_back(j);
				faces_vec.push_back(k);

			}
			f = Eigen::Map<iMATRIX>(faces_vec.data(), f_num, 3);
		}

		//read uv and uv_faces
		fMATRIX uv;
		iMATRIX uv_f;
		{
			auto uv_num = mesh->GetTextureUVCount();
			if (uv_num > 0)
			{
				std::vector<double> texcoord_vec(uv_num * 2);
				std::vector<int> uv_faces_vec;
				LoadUVInofrmation(mesh, texcoord_vec, uv_faces_vec);				

				uv = Eigen::Map<fMATRIX>(texcoord_vec.data(), uv_num, 2);
				uv_f = Eigen::Map<iMATRIX>(uv_faces_vec.data(), f_num, 3);
			}
			
		}

		//get global transformation
		auto global_tmat = get_global_transformation(node);

		TriangularMesh::init_with_vertex_face(out_mesh, v, f, &uv, &uv_f);
		out_mesh.set_transmat(global_tmat);
	}

}

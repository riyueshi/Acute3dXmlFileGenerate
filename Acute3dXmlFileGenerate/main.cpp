#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "filesystemSimplified/file_system.hpp"

using namespace std;

namespace Params {
	string xmlFileName;
	string imageRootPath;
	vector<string> groupName;
	string coordinateType;
	string imageType;
	int imageWidth;
	int imageHeight;
}

bool InitialParams()
{
	using namespace Params;
	xmlFileName = "test.xml";
	{
		groupName.push_back("CamA");
		groupName.push_back("CamB");
		groupName.push_back("CamC");
		groupName.push_back("CamD");
		groupName.push_back("CamE");
	}
	coordinateType = "EPSG:32630";
	imageRootPath = "F:/GitHub/Acute3dXmlFileGenerate/Acute3dXmlFileGenerate/data";
	imageType = ".jpg";
	imageWidth = 8176;
	imageHeight = 6132;
	return true;
}

bool ReadInPMat(const vector<vector<string>> pMatFileNames, vector < vector<cv::Matx<double, 3, 4>>> &pMats, bool ignoreFirstLine)
{
	for (int groupIndex = 0; groupIndex < pMatFileNames.size(); groupIndex++)
	{
		pMats.at(groupIndex).resize(pMatFileNames.at(groupIndex).size());
		for (int imageIndex = 0; imageIndex < pMatFileNames.at(groupIndex).size(); imageIndex++)
		{
			string fileName = Params::imageRootPath + "/" + Params::groupName.at(groupIndex) + "/" + pMatFileNames.at(groupIndex).at(imageIndex);
			if (!stlplus::file_exists(fileName))
			{
				cout << "Error: the file :" + fileName << "does not exist" << endl; getchar();
			}

			ifstream reader(fileName);
			if (ignoreFirstLine)
			{
				getline(reader, string());
			}
			reader >> pMats.at(groupIndex).at(imageIndex)(0, 0)
				>> pMats.at(groupIndex).at(imageIndex)(0, 1)
				>> pMats.at(groupIndex).at(imageIndex)(0, 2)
				>> pMats.at(groupIndex).at(imageIndex)(0, 3)
				>> pMats.at(groupIndex).at(imageIndex)(1, 0)
				>> pMats.at(groupIndex).at(imageIndex)(1, 1)
				>> pMats.at(groupIndex).at(imageIndex)(1, 2)
				>> pMats.at(groupIndex).at(imageIndex)(1, 3)
				>> pMats.at(groupIndex).at(imageIndex)(2, 0)
				>> pMats.at(groupIndex).at(imageIndex)(2, 1)
				>> pMats.at(groupIndex).at(imageIndex)(2, 2)
				>> pMats.at(groupIndex).at(imageIndex)(2, 3);
			reader.close();
		}
	}
	return true;
}

int main()
{	
	InitialParams();
	vector<vector<string>> pMatFileNames(Params::groupName.size());
	for (int groupIndex = 0; groupIndex < Params::groupName.size(); groupIndex++)
	{
		string folderName = Params::imageRootPath + "/" + Params::groupName.at(groupIndex);
		if (!stlplus::folder_exists(folderName))
		{
			cout << "Error: the folder :" + folderName << "does not exist" << endl; getchar();
		}
		else
		{
			pMatFileNames.at(groupIndex) = stlplus::folder_files(folderName);
		}
	}
	vector < vector<cv::Matx<double, 3, 4>>> pMats(Params::groupName.size());
	ReadInPMat(pMatFileNames, pMats, true);


	cv::FileStorage fs(Params::xmlFileName, cv::FileStorage::WRITE,"utf-8");
	fs << "BlocksExchange" << "{";
	fs << "SpatialReferenceSystems" << "{";
	fs << "SRS" << "{";
	fs << "Id" << 0;
	fs << "Name" << "WGS84China";
	fs << "Definition" << Params::coordinateType;
	fs << "}";//end SRS
	fs << "}";//end SpatialReferenceSystems
	fs << "BaseImagePath" << Params::imageRootPath;

	fs << "Block" << "{";
	fs << "Name" << "Block12Test";
	fs << "Description" << "this a test block";
	fs << "Type" << "Aerial";
	fs << "SRSId" << 0;
	fs << "Photogroups" << "{";

	int photoIndexGlobal(0);
	for (int photoGroupIndex = 0; photoGroupIndex < Params::groupName.size(); photoGroupIndex++)
	{
		fs << "Photogroup" << "{";
		fs << "Name" << Params::groupName.at(photoGroupIndex);
		fs << "ImageDimensions" << "{";
		fs << "Width" << Params::imageWidth;
		fs << "Height" << Params::imageHeight;
		fs << "}";//end ImageDimensions

		//depose the first project matrix of this group
		cv::Matx33d K, R;
		cv::Matx41d C;
		cv::decomposeProjectionMatrix(pMats.at(photoGroupIndex).at(0), K, R, C);

		fs << "FocalLengthPixels" << K(0,0);
		fs << "CameraOrientation" << "XRightYDown";
		fs << "PrincipalPoint" << "{";
		fs << "x" << K(0,2);
		fs << "y" << K(1,2);
		fs << "}";//end PrincipalPoint
		fs << "Distortion" << "{";
		fs << "K1" << 0.0 << "K2" << 0.0 << "K3" << 0.0 << "P1" << 0.0 << "P2" << 0.0;
		fs << "}";//end Distortion

		for (int photoIndexInGroup = 0; photoIndexInGroup < pMatFileNames.at(photoGroupIndex).size(); photoIndexInGroup++)
		{
			string imageName = stlplus::basename_part(pMatFileNames.at(photoGroupIndex).at(photoIndexInGroup)) + Params::imageType;
			fs << "Photo" << "{";
			fs << "Id" << photoIndexGlobal++;
			fs << "ImagePath" << Params::imageRootPath + imageName;

			cv::Matx33d K, R;
			cv::Matx41d C;
			cv::decomposeProjectionMatrix(pMats.at(photoGroupIndex).at(photoIndexInGroup), K, R, C);

			fs << "Pose" << "{";
			fs << "Rotation" << "{";
			fs << "M_00" << R(0,0) << "M_01" << R(0, 1) << "M_02" << R(0, 2)
				<< "M_10" << R(1, 0) << "M_11" << R(1, 1) << "M_12" << R(1, 2)
				<< "M_20" << R(2, 0) << "M_21" << R(2, 1) << "M_22" << R(2, 2);
			fs << "}";//end Rotation
			fs << "Center" << "{";
			fs << "x" << C(0) / C(3);
			fs << "y" << C(1) / C(3);
			fs << "z" << C(2) / C(3);
			fs << "}";//end Center
			fs << "}";//end Pose
			fs << "}";//end Photo
		}
		fs << "}";//end Photogroup
	}
	fs << "}";//end Photogroups
	fs << "}";//end Block
	fs << "}";//end BlocksExchange
	fs.release();
	return 0;
}
#pragma once
#include "BundleAdjustmentSettings.h"
#include "ImageDataContainer.h"
#include "ImagePointContainer.h"
#include "TerrainPointContainer.h"
#include "AngularSurvey3DContainer.h"

namespace ba
{



	class BundleAdjustmentData
	{
	public:
		std::string FilenameSettings;
		BundleAdjustmentSettings Settings;
		ImagePointContainer ImagePoints;
		TerrainPointContainer ObjectPoints;
		TerrainPointContainer GeodeticControllPoints;
		TerrainPointContainer ObjectPointsMeasurements;
		TerrainPointContainer CheckPoints;
		ImageDataContainer ImageOrientationData;
		AngularSurvey3DContainer GeodeticAngularMeasurements;

		unsigned int NumOfCameras{ 0 };
		unsigned int NumOfImages{ 0 };
		unsigned int NumOfControllPoints{ 0 };
		unsigned int NumOfCheckPoints{ 0 };
		unsigned int NumOfTiePoints{ 0 };
		unsigned int NumOfImagePoints{ 0 };


		bool read_data()
		{
			std::cout << "reading input data..." << std::endl;
			
			if (Settings.InputFiles.FilenameGeodeticClotrollPoints == "" && Settings.MathModel == BAMathModel::TIGHT)
			{
				Settings.MathModel = BAMathModel::SOFT;
				std::cout << "WARNING! Geodetic controll not deffined. Switching to SOFT Adjustment Model\n" << std::endl;
			}

			if (Settings.InputFiles.FilenameGeodeticMeasurements == "" && Settings.MathModel == BAMathModel::TIGHT)
			{
				Settings.MathModel = BAMathModel::SOFT;
				std::cout << "WARNING! Geodetic controll not deffined. Switching to SOFT Adjustment Model\n" << std::endl;
			}


			bool res = 
				Settings.read_from_file(FilenameSettings) &&
				ImagePoints.read_from_file(Settings.InputFiles.FilenameImagePoints) &&
				ObjectPoints.read_from_file(Settings.InputFiles.FilenameObjectPoints) &&
				ImageOrientationData.read_from_file(Settings.InputFiles.FilenameExternalOrientation);

			//removing image point measerumnets with no corresponding image;
			std::set<string> valid_image_ids;
			for (auto &i : ImageOrientationData.DataImages) valid_image_ids.insert(i.second.Name);
			ImagePoints.filter_out(valid_image_ids);


			if (Settings.MathModel == BAMathModel::TIGHT)
			{
				res = res&& GeodeticAngularMeasurements.read_from_file(Settings.InputFiles.FilenameGeodeticMeasurements);
				res = res && GeodeticControllPoints.read_from_file(Settings.InputFiles.FilenameGeodeticClotrollPoints);
			}


			if (res)
			{
				//removing object points that are not measured in the images
				if (Settings.MathModel == BAMathModel::TIGHT) //copying Geodetic Controll to Object Points
				{
					for (auto &p : GeodeticControllPoints.Data)
					{
						ObjectPoints.Data.emplace(p.first, p.second);
					}
				}


				if (Settings.MathModel == BAMathModel::SOFT || Settings.MathModel == BAMathModel::TIGHT)
				{
					std::vector<string> keys;
					for (auto &p : ObjectPoints.Data)
					{
						if (p.second.Type == 0 || p.second.Type == 3 || p.second.Type == 4) //tie points, controll points and check points
						{
							if (std::count_if(ImagePoints.Data.begin(), ImagePoints.Data.end(), [&p](ImagePoint ip) {return ip.Name == p.second.Name; }) == 0)
							{
								keys.push_back(p.second.Name);
								std::cout << "WARNING! Object point: " << p.second.Name << " is in input data but was not measured in any image and will not be considered!" << std::endl;
							}
						}
					}
					for (auto &k : keys) ObjectPoints.Data.erase(k);

				}

				if (Settings.MathModel == BAMathModel::SOFT)
				{
					
					//removing geodetic controll points
					std::vector<string> keys;
					for (auto &p : ObjectPoints.Data)
					{
						if (p.second.Type == 9 ) //erasing only tie points and controll points
						{
							keys.push_back(p.second.Name);
							std::cout << "WARNING! Object point: " << p.second.Name << " is an geodetic controll and will not be considered!" << std::endl;
						}
					}
					for (auto &k : keys) ObjectPoints.Data.erase(k);
				}

				//copy the whole container of object point data as we have to distinguish between object points as
				//parameters and object poits' measurements
				if (Settings.MathModel == BAMathModel::SOFT)
				{
					ObjectPointsMeasurements.Data.clear();
					for (auto &p : ObjectPoints.Data)
					{
						if (p.second.Type == 3)
						{
							ObjectPointsMeasurements.Data.emplace(p.first, p.second);
						}
					}	
				}

				//copying check points
				for (auto &p : ObjectPoints.Data)
				{
					if (p.second.Type == 4)
					{
						CheckPoints.Data.emplace(p.first, p.second);
					}
				}

				if (Settings.MathModel == BAMathModel::TIGHT)
				{
					//Object points without image rays have already been removed
					//Now we have to remove GeodeticAngularMeasurements that are not in the at least one image
					std::vector<AngularSurvey3D> Temp;
					Temp.reserve(GeodeticAngularMeasurements.Data.size());
					for (auto &d : GeodeticAngularMeasurements.Data)
					{
						if (std::count_if(ImagePoints.Data.begin(), ImagePoints.Data.end(), [&d](ImagePoint ip) {
							return (ip.Name == d.Ids[0]) || (ip.Name == d.Ids[2]); }
						) == 0)
						{
							std::cout << "WARNING! Angular survey: " << d.Ids.at(0) <<" "<<d.Ids.at(1) <<" "<<d.Ids.at(2) << " from station id = " << d.Ids[1] << " has no corresponding image projection an will be deleted!" << std::endl;
						}
						else
						{
							Temp.push_back(d);
						}
					}
					Temp.shrink_to_fit();
					//replacing geodetic angle measurement data:
					GeodeticAngularMeasurements.Data = Temp;


					//determining if left or right point of angle is to be adjusted	
					for (auto&d : GeodeticAngularMeasurements.Data)
					{
						d.AdjustedPointId = -1;
						try
						{
							
							if (ObjectPoints.Data.at(d.Ids.at(1)).Type != 9)
							{
								std::cout << "Error in angle measurement " << d.Ids.at(0) << " " << d.Ids.at(1) << " " << d.Ids.at(2) << std::endl;
								std::cout << "Point " << d.Ids.at(1) << " is not deffined as the geodetic controll!" << std::endl;
								return false;
							}
							

							if ((ObjectPoints.Data.at(d.Ids.at(0)).Type != 9) && (ObjectPoints.Data.at(d.Ids.at(2)).Type != 9))
							{
								std::cout << "Error in angle measurement '" << d.Ids.at(0) << "' '" << d.Ids.at(1) << "' '" << d.Ids.at(2) <<"'" << std::endl;
								std::cout << "One of points: " << d.Ids.at(0) << " or " << d.Ids.at(2) << " has to be a geodetic controll" << std::endl;
								return false;
							}
							
							for (int i : {0, 1, 2})
							{
								if (ObjectPoints.Data.at(d.Ids.at(i)).Type == 3 ||
									ObjectPoints.Data.at(d.Ids.at(i)).Type == 0 ||
									ObjectPoints.Data.at(d.Ids.at(i)).Type == 4 ) //measurements to check points are include as well
								{
									d.AdjustedPointId = i;
									//ADD EXCEPTION HANDLING!!!:
									//EACH POINT HAS TO BE IN THE OBJECT POINT DATASET!
								}
							}
							
						}
						catch (std::exception& e)
						{
							std::cerr << "You are trying to use the angle measurement to the point that is not in the object point dataset, " << std::endl;
							std::cerr << "or was deleted because of no corresponding image projection." << std::endl;
							std::cerr << "This is an error!!! Check the following observation:" << std::endl;
							std::cerr << "left point: '" << d.Ids[0] << "' station: '" << d.Ids[1] << "' right point: '" << d.Ids[2] <<"'" << std::endl;
							std::cerr << e.what() << std::endl;
							return false;
						}

						if (d.AdjustedPointId == -1)
						{
							std::cout << "Error in angle measurement " << d.Ids.at(0) << " " << d.Ids.at(1) << " " << d.Ids.at(2) << std::endl;
							std::cout << "This is neigher the measurement to the conroll point nor to the tie point" << std::endl;
							return false;
						}

					}


					//determining if each point has 2 angular measurements
					for (auto &d : GeodeticAngularMeasurements.Data)
					{
						if (std::count_if(
							GeodeticAngularMeasurements.Data.begin(),
							GeodeticAngularMeasurements.Data.end(),
							[&d](AngularSurvey3D a) {return a.Ids[a.AdjustedPointId] == d.Ids[d.AdjustedPointId]; }) >=2 )
						{
							
						}
						else
						{
							std::cout << "Error in angle measurement " << d.Ids.at(0) << " " << d.Ids.at(1) << " " << d.Ids.at(2) << std::endl;
							std::cout << d.Ids[d.AdjustedPointId] << " point should be provided the measruements from two geodetic controll points" << std::endl;
							return false;
						}
					}

					
				}

				//calculating number of rays for object points:
				for (auto &p : ObjectPoints.Data)
				{
					p.second.Rays = std::count_if(
						ImagePoints.Data.begin(), ImagePoints.Data.end(), [&p](ImagePoint ip) {return ip.Name == p.second.Name; });
				}

				//calculating number of points in each image:
				for (auto &i : ImageOrientationData.DataImages)
				{
					i.second.NumOfPoints = std::count_if(
						ImagePoints.Data.begin(), ImagePoints.Data.end(), [&i](ImagePoint ip) {return ip.ImageName == i.second.Name; });
				}

				for (auto& p : ImagePoints.Data) //rewriting image measurement accuracy
				{
					p.MX = Settings.ImageMesAcc;
					p.MY = Settings.ImageMesAcc;
				}
				
				NumOfImages = ImageOrientationData.DataImages.size();
				NumOfCameras = ImageOrientationData.DataCameras.size();
				NumOfControllPoints = ObjectPoints.Data.size();
				NumOfImagePoints = ImagePoints.Data.size();
				NumOfCheckPoints = CheckPoints.Data.size();
			}

			return res;
		}

		void scale()
		{
			for (auto &ip : ImagePoints.Data)
			{
				ip.scale(
					ImageOrientationData.DataCameras.at(
						ImageOrientationData.DataImages.at(ip.ImageName).CameraName).DataScalling
				);
			}

			for (auto &c : ImageOrientationData.DataCameras)
			{
				c.second.scale();
			}
		}

		void scale_back()
		{
			for (auto &ip : ImagePoints.Data)
			{
				ip.scale_back(
					ImageOrientationData.DataCameras.at(
						ImageOrientationData.DataImages.at(ip.ImageName).CameraName).DataScalling
				);
			}

			for (auto &c : ImageOrientationData.DataCameras)
			{
				c.second.scale_back();
			}

		}


	}; //class BundleAdjustmentData

}//namespace ba
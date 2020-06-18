#include <iostream>
#include <cstdlib>
#include <string>
#include <liblas/liblas.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char *argv[])
{
	std::cout << "===================================================================" << std::endl;
	std::cout << "LAS2PCD - Converts .las point clouds into PCL-friendly format .pcd" << std::endl;
	std::cout << "ver 0.3 - 16 March 2018" << std::endl;
	std::cout << "(c) Arnadi Murtiyoso" << std::endl;
	std::cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << std::endl;
	std::cout << "contact: arnadi.murtiyoso@insa-strasbourg.fr" << std::endl;
	std::cout << "https://github.com/murtiad" << std::endl;
	std::cout << "Ubuntu tweaks by Jonathan Greenberg, jgreenberg@unr.edu" << std::endl;
	std::cout << "Ubuntu version: https://github.com/gearslaboratory/las2pcd" << std::endl;
	std::cout << "===================================================================" << std::endl;
	std::cout << std::endl;

    if (argc != 3)
    {
        std::cerr << "You must enter an input path and an output path" << std::endl;
        return 1;
    }

    std::string inPath = argv[1];
    std::string outPath = argv[2];

    std::cerr << "INFO : Loading : " << inPath << std::endl;

    // instancing a new PCL pointcloud object
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Opening  the las file
    std::ifstream ifs(inPath, std::ios::in | std::ios::binary);

    // Safeguard against opening failure
    if(ifs.fail())
    {
        std::cerr << "ERROR : Impossible to open the file : " << inPath << std::endl;
        return 1;
    }

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs); // reading las file
    unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();

	// Fill in the cloud data
	cloud.width    = nbPoints;				// This means that the point cloud is "unorganized"
	cloud.height   = 1;						// (i.e. not a depth map)
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	std::cout << "INFO : " << cloud.points.size () << " points detected in " << inPath << std::endl;

	int i=0;				// counter
	uint16_t r1, g1, b1;	// RGB variables for .las (16-bit coded)
	int r2, g2, b2;			// RGB variables for converted values (see below)
	uint32_t rgb;			// "packed" RGB value for .pcd

	while(reader.ReadNextPoint())
	{
		// get XYZ information
		cloud.points[i].x = (reader.GetPoint().GetX());
	        cloud.points[i].y = (reader.GetPoint().GetY());
	        cloud.points[i].z = (reader.GetPoint().GetZ());

		// get RGB information. Note: in liblas, the "Color" class can be accessed from within the "Point" class, thus the triple gets
		r1 = (reader.GetPoint().GetColor().GetRed());
		g1 = (reader.GetPoint().GetColor().GetGreen());
		b1 = (reader.GetPoint().GetColor().GetBlue());

		// .las stores RGB color in 16-bit (0-65535) while .pcd demands an 8-bit value (0-255). Let's convert them!
		r2 = ceil(((float)r1/65536)*(float)256);
		g2 = ceil(((float)g1/65536)*(float)256);
		b2 = ceil(((float)b1/65536)*(float)256);


		// PCL particularity: must "pack" the RGB into one single integer and then reinterpret them as float
		rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);

		cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);

		i++; // ...moving on
	}

	// Allows output file to be set:
        pcl::io::savePCDFileASCII (outPath, cloud);

	std::cerr << "Saved " << cloud.points.size () << " data points to pointcloud.pcd." << std::endl;

	return (0);
}

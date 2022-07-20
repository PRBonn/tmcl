#include <iostream>
#include "Python.h"
#include <string>
#include <vector>
#include <utility>

#include <eigen3/Eigen/Dense>



enum FrameTypes
{
	CAMERA0 = 0,
	CAMERA1 = 1,
	CAMERA2 = 2,
	CAMERA3 = 3,
	SEM0 = 4,
	SEM1 = 5,
	SEM2 = 6,
	SEM3 = 7,
	TEXT0 = 8,
	TEXT1 = 9,
	TEXT2 = 10,
	TEXT3 = 11,
	LIDAR = 12,
	ODOM = 13,
	GT = 14
};



struct FrameData
{
	FrameTypes type;
	unsigned long stamp;
	Eigen::Vector3f gt;
    std::string path;
    Eigen::Vector3f odom;
    std::vector<Eigen::Vector3f> scan;
    std::vector<std::string> places;
    std::vector<Eigen::Vector4f> boxes;
};



class DataFrameLoader
{
	public:
		DataFrameLoader(const std::string& moduleFolder, const std::string& moduleName);

		void Load(const std::string& picklePath);

		FrameData GetData(unsigned long index);

		int GetNumFrames();


		~DataFrameLoader();


	private:

		const char * get_type(unsigned long ind);
		unsigned long get_stamp(unsigned long ind);
		PyObject * get_data(unsigned long ind);
		static std::vector<float> listTupleToVector_Float(PyObject* incoming); 


		PyObject *pDict;
		PyObject *dfObject;

};


%module ProcessArcher
%include "typemaps.i"
%{
#include "ProcessArcher.h"
%}

%exception get_Data {
	try {
		$action
	} catch (std::out_of_range &e) {
		PyErr_SetString(PyExc_IndexError, const_cast<char*>(e.what()));
		return NULL;
	}
}


%apply double *OUTPUT { double *ground_x };
%apply double *OUTPUT { double *ground_y };

class ArcherINSData {
public:
	ArcherINSData();
	~ArcherINSData();
	void setParams(double FOV, double RollBias, double PitchBias, double HeadingBias, 
					    double LongitudeBias, double LatitudeBias, double AltitudeBias, int Samples);

	void append(int sequence, double roll, double pitch, double yaw, 
			double lon, double lat, double alt, double time, 
			double x, double y, double heading);
	
	void set_bbox(double minx, double miny, double maxx, double maxy);
	double* get_bbox();

	void image_to_ground(int img_x, int img_y, double *ground_x, double *ground_y);
			
	ArcherINSRow* get_Data();
	ArcherINSRow  get_Data(int row);
	int           length();
};

class ProcessArcher {
public:
	ProcessArcher(char* file_in, char* file_out, ArcherINSData *ins);
	~ProcessArcher();
	
	int run(double meters_per_pixel);
};


const char* version();

%typemap(in) double[ANY](double temp[$1_dim0]) {
  int i;
  if (!PySequence_Check($input)) {
      PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
      return NULL;
  }
  if (PyObject_Length($input) != $1_dim0) {
      PyErr_SetString(PyExc_ValueError,"Expecting a sequence with $1_dim0 elements");
      return NULL;
  }
  for (i =0; i < $1_dim0; i++) {
      PyObject *o = PySequence_GetItem($input,i);
      if (!PyFloat_Check(o)) {
         Py_XDECREF(o);
         PyErr_SetString(PyExc_ValueError,"Expecting a sequence of floats");
         return NULL;
      }
      temp[i] = PyFloat_AsDouble(o);
      Py_DECREF(o);
  }
  $1 = &temp[0];
}

%apply double *OUTPUT { double *img_x };
%apply double *OUTPUT { double *img_y };
void ground_to_output(double gt[6], double ground_x, double ground_y, double *img_x, double *img_y);
%apply int *OUTPUT { int *img_x };
%apply int *OUTPUT { int *img_y };
void ground_to_output_int(double gt[6], double ground_x, double ground_y, int *img_x, int *img_y);


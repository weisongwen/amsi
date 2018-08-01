#pragma once

// customized head file
#include "rtklib/rtklib.h"
//#include "WeightLeastSquare.h"
//#include "SelectEphemeris.h"

// eigen 
#include "Eigen/Eigen/Eigen"

// vector, io stream
#include <vector>
#include <iostream>

// fstream
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
//time 
#include <time.h>
#include <dos.h> //for delay
//algorithm 
#include <algorithm>
#include <omp.h>
// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000
using namespace Eigen;
using namespace std;

//#define TARGETWEEK 1997
//#define STARTTIME 218012
//#define ENDTIME   218014


class shadowMatching
{
public:
	shadowMatching();
	~shadowMatching();

public: // variables for particles
	double reserve;
	typedef struct obs_Nav_epoch // reserved
	{

	};

	typedef struct  // satellite information
	{
		double GNSS_time;
		double total_sv; // total satellite in this epoch
		double prn_satellites_index; // satellite prn 
		double pseudorange; // satellite pseudorange
		double snr; // satellite snr
		double elevation; // satellite elevation 
		double azimuth; // satellite azimuth 
		double err_tropo; //satellite erro_tropo
		double err_iono; // satellite ono
		double sat_clk_err; // satellite clock bias 
		double sat_pos_x; // satellite position x
		double sat_pos_y; // satellite position y
		double sat_pos_z; // satellite position z
		int visable; // satellite visability
		std::string sat_system; // satellite system ("GPS", "GLONASS")
	}satelliteInfo;

	typedef struct // single grid: determined in ENU coordiante system
	{
		double E_;
		double N_;
		double U_;
		double score;
		//
	}grid;

	typedef struct  //  grid sequences (grids)
	{
		std::vector<grid> grids_;
	}grids;

	typedef struct  // state for a particle
	{
		double GNSS_time = 0; // time stamps
							  //satellites information 
		std::vector<satelliteInfo>  satInfo; // incluide pseudorange.etc
		int particle_ID; // the ID for particle
		double scores; // scores for this particle 
					   //position in llh
		
		double lon; // latitude 
		double lat; // longtutude 
		double altitude; // altitude 


		grids smGrids; 	//grid at present for shadow matching (plane)
		std::vector<grids> smGridss; // all the grids from first epoch to last epoch (multi-plane)

		//position in ENU
		double E;
		double N;
		double U;
		double ini_lon;
		double ini_lat;
		double ini_alt;

		// related measurements
		MatrixXd eAllMeasurement;
		MatrixXd eELAZAngles; // all the possible azimuth 
		vector<int> visibility;

	}particle;
	std::vector<particle> particles; // all the particles at present
	std::vector<std::vector<particle>> particless; // all the particles from first epoch to last epoch

public:
	int reserved = 0; // not used yet

public:  // for generate map
	typedef struct
	{
		double lon;
		double lat;
		double alt;
	}LLH;
	typedef double coord_t;         // coordinate type
	typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2
	struct Point {
		coord_t x, y;

		bool operator <(const Point &p) const {
			return x < p.x || (x == p.x && y < p.y);
		}
	};
	typedef struct
	{
		double E;
		double N;
		double U;
	}ENU;
	typedef struct // struct for a processed building information
	{
		double center_E; // center position of the building in E direction
		double center_N; // center position of the building in N direction
		double center_U; // center position of the building in U direction
		double ini_lon; // initial reference position for ENU
		double ini_lat; // initial reference position for ENU
		double ini_alt; // initial reference position for ENU
		vector <LLH> buildingLLHV; // building node list (each node with respect to a vertex) in llh
		vector <ENU> buildingENUV; // building node list (each node with respect to a vertex) in ENU
		vector<Point> buildingHull; // building hull points (used for later check if the point is inside the building) in ENU
		double sAzimuth; // smallest azimuth
		double bAzimuth; // bigest azimuth


	} building;
	vector<building> buildingS; // processed building information
	typedef struct  // struct a map
	{
		vector<building> buildingS_M; // processed building information save in a map struct
									  // lon lat alt boundary 
		double lonMax; // max longitude
		double latMax; // max latitude
		double altMax; // max altitude
		MatrixXd llhMax = MatrixXd::Random(3, 1); // available 

		double lonMin; // min longitude
		double latMin; // min latitude
		double altMin; // min altitude
		MatrixXd llhMin = MatrixXd::Random(3, 1); // available 

												  // ENU boundary
		double EMax; // max E
		double NMax; // max N
		double UMax; // max U
		MatrixXd ENUMax = MatrixXd::Random(3, 1); // available 

		double EMin; //  min E
		double NMin; //  min N
		double UMin; //  min U
		MatrixXd ENUMin = MatrixXd::Random(3, 1); // available 

												  // original llh
		double lonOri; // original lon of ENU (refers to reference point)
		double latOri; // original lat of ENU (refers to reference point)
		double altOri; // original alt of ENU (refers to reference point)
		MatrixXd llhOri = MatrixXd::Random(3, 1);

		// minimum elevation angle and corresponding maxinum searching distance
		double MinElevation = 15; // satellites with a elevation less than 15 degree will not be considered
		double maxSearDis; // the maximum searching distance when finding the insections
						   // azimuth search resolution 
		double aziReso = 1.0; // azimuth search resolution 
		double disReso = 2.0; // grid resolution for skymasks
	}map;
	map map_; // the map saving all the buildings

public: // for obtain sky mask
	vector<string> skysmaskPosString;
	vector<string> skysmaskString;

public:  // for obtain sky mask
	typedef struct // mask elevation
	{
		double azimuth; // the azimuth 
		double elevation; // the mask elevation
	}elevationMask;

	typedef struct
	{
		LLH posLLH; // pose in LLH
		LLH posLLHR; // pose in LLH for reference
		ENU poseENU; // pose in ENU
		double buildingNum; // how many this position can intersect: if point inside building (buildingNum =1)
		bool insideBuilding; // inside buildings (=1) , outside building (=0)
		float aziResol; // azimuth resolution 
		vector<elevationMask> aziElemask; // a vector 
	}skyMask; // the sky mask in a position (refers to the skyplot in one position)
	vector<skyMask> skyMaskS; // save all the skymsk in one vector 
	LLH posLLHR_; // pose in LLH for reference
	double scoreThres;
	Eigen::MatrixXd WSMenu_; // 

public:  // time delay
	void wait(int seconds)
	{
		clock_t endwait;
		endwait = clock() + seconds * CLOCKS_PER_SEC;
		while (clock() < endwait) {}
	}

public: // functions 
	bool readSkymask(string filepath); // read the skymask file 
	void string2llh(vector<string>, vector<string>); // extract the elevation from the string for azimuth,elevation...
	skyMask preparingSkymaskbyPose(ENU); // prepare a skymask by the ENU coordinate 
	vector<int> visibilityCal(MatrixXd, skyMask); // calculate the expected visibility
	double scoreCal(MatrixXd, vector<int>); // calculate the scores of shadowmatching 
	vector<particle> generateParticles(Eigen::MatrixXd WLSECEF,double radis, double resol, Eigen::MatrixXd eAllMeasurement, Eigen::MatrixXd eELAZAngles); // Generate particle near the WLS solution
	MatrixXd MajorAxisCor(MatrixXd, MatrixXd, MatrixXd); // solution correction based on major axis 
	double MajorAxisScoreCal(vector<int>, vector<int>); // calculate score (major axis)
	MatrixXd weightSM_(void); // weight all the particles and obtain the final solution
	//void prepareSatelliteInfo(vector<vector <LLH>>); // prepare the satellites informations : do some pre-processing
									
	// coordiante system transform 
	Eigen::MatrixXd llh2ecef(Eigen::MatrixXd); // transform the llh to ecef
	Eigen::MatrixXd ecef2llh(Eigen::MatrixXd); // transform the ecef to llh 
	Eigen::MatrixXd ecef2enu(Eigen::MatrixXd, Eigen::MatrixXd ); // transform the ecef to enu
	Eigen::MatrixXd enu2ecef(Eigen::MatrixXd, Eigen::MatrixXd); // transform the enu to ecef



};





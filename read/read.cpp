#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <math.h>

using namespace std;

// Translation from python file, basically returns a cleaned up format
// of the CSV file that the kraken outputs
struct csvResult{
	vector<double> lat_list;
	vector<double> long_list;
	vector<int> bearing_list;
	double maxConfidence = 0;
};

// Coordinate structure, only has two variables; mainly did this for
// the sake of making it easier to pass through as a param/return
struct coordinate{
	double latitude = 0;
	double longitude = 0;
};

csvResult readFile(string path);
coordinate findFinal(csvResult result);
double findDistance(coordinate initial, coordinate final);
coordinate findNew(coordinate initial, int angle, double displacement);

int main(){
// WILL CHANGE THIS OUT WHEN WE DO FLIGHT TESTING, THIS IS JUST FOR
// LOCAL TESTING
	string path = "test.csv";
// UP TO HERE NEEDS TO BE SET TO A FIXED SPOT IN THE DIRECTORY
	csvResult result = readFile(path);
	coordinate point = findFinal(result);
	cout << fixed << setprecision(8);
	cout << point.latitude << " " << point.longitude << endl;
	return 0;

}

// reads a csv file from the kraken, then spits out a list of coordinates
csvResult readFile(string path){
	csvResult result;
	ifstream liveCSV(path);

	// Error checking, either the file does not exist, or wrong path
	if (!liveCSV.is_open()){
		cerr << "\aError opening file\n\n";
		exit(EXIT_FAILURE);
	} else {
		int index = 0;
		int accuracyIndex = 0;
		string line;

		while (getline(liveCSV, line)){
			istringstream stream(line);
			string token;
			char delimiter = ',';
			vector<string> parse;
			while (getline(stream, token, delimiter)){
				parse.push_back(token);
			}
			double latitude = stod(parse[8]);
			double longitude = stod(parse[9]);
			double confidence = stod(parse[2]);
			int bearing_degree = stoi(parse[1]);
			result.lat_list.push_back(latitude);
			result.long_list.push_back(longitude);
			result.bearing_list.push_back(bearing_degree);
			if (result.maxConfidence < confidence){
				result.maxConfidence = confidence;
			}
		}
	}
	liveCSV.close();
	return result;
}

coordinate findFinal(csvResult result){
	vector<coordinate> finalPoints;
	vector<double> latList = result.lat_list;
	vector<double> longList = result.long_list;
	vector<int> angleList = result.bearing_list;
	for (int i = 1; i < latList.size(); i++){
		coordinate initial, final;
		initial.latitude = latList[i-1];
		initial.longitude = longList[i-1];
		final.latitude = latList[i];
		final.longitude = longList[i];
		double displacement = findDistance(initial, final);
		int angle = angleList[i];
		coordinate newCoord = findNew(final, angle, displacement);
		finalPoints.push_back(newCoord);
	}
	if (result.maxConfidence < 80){
		cout << "LOW CONFIDENCE" << endl;
	}
	coordinate finalPoint;
	for (int i = 0; i < finalPoints.size(); i++){
		finalPoint.latitude += finalPoints[i].latitude;
		finalPoint.longitude += finalPoints[i].longitude;
	}
	finalPoint.latitude = finalPoint.latitude/finalPoints.size();
	finalPoint.longitude = finalPoint.longitude/finalPoints.size();
	return finalPoint;
}

double findDistance(coordinate initial, coordinate final){
	double latDisplacement = initial.latitude - final.latitude;
	double longDisplacement = initial.longitude - final.longitude;
	double num = sqrt(pow(latDisplacement, 2) + pow(longDisplacement, 2));
	return num;
}

coordinate findNew(coordinate initial, int angle, double displacement){
	double radians = angle * M_PI / 180.0;
	coordinate newCoordinate;
	newCoordinate.latitude = initial.latitude + (displacement * cos(radians));
	newCoordinate.longitude = initial.longitude + (displacement * sin(radians));
	return newCoordinate;
}


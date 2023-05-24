#include "isodata.h"
#include "MyTime.h"
#include <sstream>

/**
 * to read the data stored in a .txt file with one point per line
 * within a line, characteristics are separated by ',', no special end-of-line character
 * @return data a vector of points, the latter themselves being vectors of double
 */
vector<vector<double>> read_data(const char* filePath)
{
  ifstream is;
  is.open(filePath);
  if (!is.is_open())
    {
      cout << WARN_FILE_OPEN_FAIL << endl;
    }
  vector<vector<double>> data;
  int id(0);
  while (!is.eof())
    {
      string s;
      getline(is, s);
      stringstream ss(s);
      vector<double> line;
      double d;
      char c;
      while (!ss.eof() && ss >> d)
        {
	  line.emplace_back(d);
	  ss >> c;

        }
      if (!line.empty())
	data.emplace_back(line);
    }
  data.shrink_to_fit();
  is.close();
  return data;
}

/**
 * function read_data to personalize above
 */
int main(int argc, char* argv[]) { // args : dataFile
  // TO RUN WITH POINTS
  int samples, sampleSize;
  Eigen::MatrixXd points = read_data(argv[1], samples, sampleSize);
  isodata isodata2(points, samples, sampleSize, 4, 90, 10, 90, 20, 5, 500);
  std::vector<ResultCluster> res = isodata2.runWPoints();
  std::cout << "nb clusters : " << res.size() << std::endl;
  std::cout << "cluster 1 \nsize : " << res[0].points.rows() << std::endl;
  std::cout << "centroid :\n " << res[0].centroid << std::endl;
  // std::cout << "points " << res[0].points << std::endl;
  return 0;
}



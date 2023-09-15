#include "isodata.h"
#include <sstream>

/**
 * to read the data stored in a .txt file with one point per line
 * within a line, characteristics are separated by ',', no special end-of-line character
 * @return data a vector of points, the latter themselves being vectors of double
 */
Eigen::MatrixXd read_data(const char* filePath, int &rows, int &cols)
{
  std::ifstream is;
  is.open(filePath);
  if (!is.is_open())
    {
      std::cout << WARN_FILE_OPEN_FAIL << std::endl;
    }
  std::vector<std::vector<double>> Vdata;
  int id(0);
  while (!is.eof())
    {
      std::string s;
      std::getline(is, s);
      std::stringstream ss(s);
      std::vector<double> line;
      double d;
      char c;
      while (!ss.eof() && ss >> d)
        {
	  line.emplace_back(d);
	  ss >> c;

        }
      if (!line.empty())
	Vdata.emplace_back(line);
    }
  Vdata.shrink_to_fit();
  is.close();
  rows = Vdata.size();
  cols = Vdata[0].size();
  Eigen::MatrixXd Mdata = Eigen::MatrixXd::Zero(rows, cols);
  for (int i=0; i<rows; i++){
    Eigen::Map<Eigen::VectorXd> pointData(Vdata[i].data(), cols);
    Mdata.row(i) = pointData;
  }
  return Mdata;
}

/**
 * function read_data to personalize above
 */
int main(int argc, char* argv[]) { // args : dataFile
  int samples, sampleSize;
  Eigen::MatrixXd points = read_data(argv[1], samples, sampleSize);
  isodata isodata2(points, samples, sampleSize, 4, 5, 1, 1., .75, 5, 500, 3.);
  std::vector<ResultCluster> res = isodata2.run();
  std::cout << "nb clusters : " << res.size() << std::endl;
  std::cout << "cluster 1 \nsize : " << res[0].points.rows() << std::endl;
  std::cout << "centroid :\n " << res[0].centroid << std::endl;
  std::cout << "cluster 2 \nsize : " << res[1].points.rows() << std::endl;
  std::cout << "centroid :\n " << res[1].centroid << std::endl;
  return 0;
}



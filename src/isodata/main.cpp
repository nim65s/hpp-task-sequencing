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
int main(int argc, char* argv[]) { // args : dataFile, paramFile, resultFile
  if (argc!=4){
    cout << "Not enough arguments (data, parameters and result files required)" << endl;
  }
  else{
    CMyTimeWrapper c;
    c.tic();
    isodata isodata1(read_data, argv[1], argv[2], argv[3]);
    isodata1.run();
    c.tocMs();
  }
  return 0;
}



#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <string.h> //memset
#include <vector>
#include <string>
#include <random>
#include <algorithm> // std::transform
#include <map>
#include <tuple>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>
#include <stdio.h>
#include "terminalinfo.h"
#include "fmat.h"
#include "environment.h"
#include <boost/format.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
/**
 * @brief Returns whether a number is positive or negative as a float +1, 0, -1
 *
 * @param x
 * @return float
 */
inline static float sign(float x)
{
  if (x > 0) { return 1; }
  if (x < 0) { return -1; }
  return 0;
}

/**
 * Increase a counter by 1, or reset to 1 if above a given limit
 *
 * @param counter The counter value (uint). This will be increased by one.
 * @param limit The limit of the counter. If this is passed. Then counter = 1.
 */
inline static void increase_counter(uint &counter, const uint &limit)
{
  if (counter > limit) {
    counter = 1;
  } else {
    counter++;
  }
}

/**
 * Increase a counter by 1, or reset to a value if above a given limit
 *
 * @param counter The counter value (uint). This will be increased by one.
 * @param limit The limit of the counter. If this is passed. Then counter = 1.
 */
inline static void increase_counter_to_value(uint &counter, const uint &limit, const uint &reset_value)
{
  if (counter > limit) {
    counter = reset_value;
  } else {
    counter++;
  }
}


/**
 * Convert an 8bit boolean vector to an unsigned integer
 * TODO: Add check for vector length
 *
 * @param t An 8 bit boolean vector
 * @return Integer value of the boolean vector
 */
inline static uint bool2int(std::vector<bool> t)
{
  uint n = 0; //Initialize
  for (uint i = 0; i < t.size(); i++) {
    n += (uint)t[i] * (uint)pow(2, t.size() - 1 - i);
  }
  return n;
}

/**
 * @brief Bind (saturate) a value between a minimum and a maximum.
 * This is basically a saturation filter.
 *
 * @param value The value of interest
 * @param min Minimum bound
 * @param max Maximum bound
 */
inline static void keepbounded(float &value, float min, float max)
{
  if (value < min) { value = min; }
  else if (value > max) { value = max; }
}

/**
 * Wrap an integer value to a sequence.
 * For instance if min = 1 and max = 8, and x = 10, then the function returns 2. Because it loops 8+2=10
 * TODO: this works only if min = 1 so adjust it
 */
inline static int wraptosequence(int x, int min, int max)
{
  if (x > max) {
    while (x > max) {
      x -= max;
    }
  } else {
    while (x < min) {
      x += max;
    }
  }
  return x;
}

/**
 * Select a random value
 * Compliments of Christopher Smith
 * https://stackoverflow.com/questions/6942273/how-to-get-a-random-element-from-a-c-container
 */
template <typename Iter, typename RandomGenerator>
inline static Iter select_randomly(Iter start, Iter end, RandomGenerator &g)
{
  std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  advance(start, dis(g));
  return start;
}

/**
 * Select a random value
 * Compliments of Christopher Smith
 * https://stackoverflow.com/questions/6942273/how-to-get-a-random-element-from-a-c-container
 */
template <typename Iter>
inline static Iter select_randomly(Iter start, Iter end)
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  return select_randomly(start, end, gen);
}

/**
 * Calculate the mean of all elements in a vector
 *
 * @param v std::vector holding the values
 */
inline static float vector_mean(const std::vector<float> &v)
{
  float sum = std::accumulate(v.begin(), v.end(), 0.0);
  return sum / v.size();
}

/**
 * Calculate the standard deviation of all elements in a vector
 *
 * @param v std::vector holding the values
 */
inline static float get_vector_std(const std::vector<float> &v)
{
  std::vector<double> diff(v.size());
  transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), vector_mean(v)));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return sqrt(sq_sum / v.size());
}

/**
 * Read a matrix from a txt file
 *
 * @param filename = name of file
 */
inline static std::vector<std::vector<float>> read_matrix(const std::string filename)
{
  std::ifstream in(filename);
  std::string line;
  std::vector<std::vector<float>> matrix;
  uint rows = 0;
  if (in.is_open()) {
    while (!in.eof()) {
      std::getline(in, line);
      std::stringstream ss(line);
      matrix.push_back(std::vector<float>());
      float value;
      while (ss >> value) {
        matrix[rows].push_back(value);
      }
      rows++;
    } 
  } else {
    std::string msg = "Matrix file not loaded " + filename;
    terminalinfo::error_msg(msg);
  }
  return matrix;
}

// object to accomodate inference of a MLP
struct Policy {
  std::vector<float> params; //single array with all weights, bias add
  std::vector<int> shape; //array with number of nodes in each layer, e.g., [3,20,20,3] means input layer with 3 nodes, 2 hidden lyaers of 20 nodes, output layer with 3 nodes
};

/**
 * Read policy parameters from txt file
 *
 * @param filename = name of file
 * @return 
 */
inline static std::vector<float> load_vector(const std::string filename)
{
  std::ifstream in(filename);
  std::string line;
  std::vector<float> vector;
  uint rows = 0;
  if (in.is_open()) {
    while (!in.eof()) {
      std::getline(in, line);
      std::stringstream ss(line);
      float value;
      while (ss >> value) {
        vector.push_back(value);
      }
      rows++;
    } 
  } else {
    std::string msg = "Matrix file not loaded " + filename;
    terminalinfo::error_msg(msg);
  }
  return vector;
}


/**
 * Read gas data points to gas object
 *
 * @param filename = name of file
 * @param first_file = boolean, if true, we read the first few lines to get some specifics about the environment
 */
inline static int load_gas_file(const std::string filename, const bool first_file, Gasdata &gas_obj)
{
  // std::ifstream in(filename);
  std::string line;
  std::vector<std::vector<float>> temp_matrix;
  // uint rows = 0;

  std::ifstream in(filename.c_str(), std::ios_base::binary);
  boost::iostreams::filtering_istream inbuf;
  inbuf.push(boost::iostreams::zlib_decompressor());
  inbuf.push(in);

  if (in.is_open()) {
    
      int row = 0;
      while (std::getline(inbuf, line)) {
        std::stringstream ss(line);
        temp_matrix.push_back(std::vector<float>());
        float value;
        std::string temp;
        while (!ss.eof()) {
          ss >> temp;
          if (std::stringstream(temp)>>value){
            temp_matrix[row].push_back(value);
            
          temp = "";
          }
          
        }
        row++;
      }
    
      if(first_file)
      {
      //we've now loaded the first 5 lines, we need to insert the values into the Gasdata object
      gas_obj.env_min = temp_matrix[0];
      gas_obj.env_max = temp_matrix[1];
      gas_obj.numcells = std::vector<int>(temp_matrix[2].begin(),temp_matrix[2].end());
      gas_obj.cell_sizes = temp_matrix[3];
      gas_obj.source_location = temp_matrix[4];
      }
    
    
    int last_row = row-1;
    std::vector<std::vector<int>> temp_gas_arr(gas_obj.numcells[0],std::vector<int> (gas_obj.numcells[1],0));
    std::vector<int> current_row;
    int max = 0;
    for (row = 7; row<last_row; row++){
      current_row = std::vector<int>(temp_matrix[row].begin(),temp_matrix[row].end());
      if (current_row[2] == 0){ //measuring gas concentration at ground level
        temp_gas_arr[current_row[0]][current_row[1]] = current_row[3];
        if (current_row[3] > max) 
        {
          max = current_row[3];
        }
      }
    }
    gas_obj.max_gas.push_back(max); //we can use this for visualization
    gas_obj.gas_data.push_back(temp_gas_arr); //store the array with all gas data at this hight

    return 1;
  }
  else{
    return 0;
  }   
}

inline static void save_as_bmp(const char* file_name, Gasdata &gas_obj, int index)
{


    FILE *f;
    unsigned char *img = NULL;
    int w = gas_obj.numcells[0], h = gas_obj.numcells[1];
    int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int
    int r,g,b,x,y;
    std::vector<std::vector<int>> data = gas_obj.gas_data[index];
    int max = gas_obj.max_gas[index];

    img = (unsigned char *)malloc(3*w*h);
    memset(img,0,3*w*h);

    for(int i=0; i<w; i++)
    {
        for(int j=0; j<h; j++)
        {
            x=i; y=(h-1)-j;
            r = (int)(data[i][j]*(255./max));
            g = r, b = r; //white = more gass
            if (r > 255) r=255;
            if (g > 255) g=255;
            if (b > 255) b=255;
            img[(x+y*w)*3+2] = (unsigned char)(r);
            img[(x+y*w)*3+1] = (unsigned char)(g);
            img[(x+y*w)*3+0] = (unsigned char)(b);
        }
    }

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(       w    );
    bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(       w>>16);
    bmpinfoheader[ 7] = (unsigned char)(       w>>24);
    bmpinfoheader[ 8] = (unsigned char)(       h    );
    bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
    bmpinfoheader[10] = (unsigned char)(       h>>16);
    bmpinfoheader[11] = (unsigned char)(       h>>24);
    
    gas_obj.bmp_header_size = sizeof(bmpfileheader)+sizeof(bmpinfoheader);
    f = fopen(file_name,"wb");
    fwrite(bmpfileheader,1,14,f);
    fwrite(bmpinfoheader,1,40,f);
    for(int i=0; i<h; i++)
    {
        fwrite(img+(w*(h-i-1)*3),3,w,f);
        fwrite(bmppad,1,(4-(w*3)%4)%4,f);
    }

    free(img);
    fclose(f);

}


inline static std::vector<std::vector<float>> read_points(const std::string filename)
{
  std::ifstream in(filename);
  std::string line;
  std::vector<std::vector<float>> matrix;
  uint rows = 0;
  if (in.is_open()) {
    while (!in.eof()) {
      std::getline(in, line);
      std::stringstream ss(line);
      matrix.push_back(std::vector<float>());
      float value;
      while (ss >> value) {
        matrix[rows].push_back(value);
      }
      rows++;
    } 
  } else {
    std::string msg = "Matrix file not loaded " + filename;
    terminalinfo::error_msg(msg);
  }
  return matrix;
}



inline static std::vector<float> read_vector(const std::string filename)
{
  std::ifstream in(filename);
  std::string line;
  std::vector<float> matrix;
  uint rows = 0;
  if (in.is_open()) {
    while (!in.eof()) {
      std::getline(in, line);
      std::stringstream ss(line);
      float value;
      while (ss >> value) {
        matrix.push_back(value);
      }
      rows++;
    } 
  } else {
    std::string msg = "Matrix file not loaded " + filename;
    terminalinfo::error_msg(msg);
  }
  return matrix;
}

// clips value between min and max
inline static int clip(int input, int min, int max)
{
  if (input>max)
  {
    return max;
  }
  else if (input <min)
  {
    return min;
  }
  else
  {
    return input;
  }
}

/**
 * Read an array from a txt file
 *
 * @param filename = name of file
 */
inline static std::vector<float> read_array(const std::string filename)
{
  std::ifstream in(filename);
  std::string line;
  std::vector<float> array;
  if (in.is_open()) {
    std::getline(in, line);
    std::stringstream ss(line);
    float value;
    while (ss >> value) {
      array.push_back(value);
    }
  } else {
    terminalinfo::error_msg("Array file not loaded: " + filename);
  }
  return array;
}

/**
 * 2D Point class use for the functions below
 *
 */
struct Point {
  float x;
  float y;
};

/**
 * Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'
 *
 * @param p
 * @param q
 * @param r
 * @return true
 * @return false
 */
inline static bool onSegment(Point p, Point q, Point r)
{
  float margin = 0.05;
  if (q.x <= (std::max(p.x, r.x)+margin) && q.x >= (std::min(p.x, r.x)-margin) &&
      q.y <= (std::max(p.y, r.y)+margin) && q.y >= (std::min(p.y, r.y)-margin)) {
    return true;
  }

  return false;
}

/**
 * Function to find orientation of ordered triplet (p, q, r).
 * The function returns following values
 * 0 --> p, q and r are colinear
 * 1 --> Clockwise
 * 2 --> Counterclockwise
 *
 * @param p
 * @param q
 * @param r
 * @return int
 */
inline static int orientation(Point p, Point q, Point r)
{
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  int val = (q.y - p.y) * (r.x - q.x) -
            (q.x - p.x) * (r.y - q.y);

  if (val == 0) { return 0; }  // colinear

  return (val > 0) ? 1 : 2; // clock or counterclock wise
}



/**
 * Returns true if line segment 'p1q1' and 'p2q2' intersect, false otherwise.
 *
 * @param p1 Start segment 1
 * @param q1 End segment 1
 * @param p2 Start segment 2
 * @param q2 End segment 2
 * @return true Lines intersect
 * @return false Lines do not intersect
 */
inline static bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
  // Find the four orientations needed for general and
  // special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4) {
    return true;
  }

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1)) { return true; }

  // p1, q1 and q2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1)) { return true; }

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2)) { return true; }

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(p2, q1, q2)) { return true; }

  return false; // Doesn't fall in any of the above cases
}



/**
 * Returns point of intersect of two lines
 *
 * @param p1 Start segment 1
 * @param q1 End segment 1
 * @param p2 Start segment 2
 * @param q2 End segment 2
 * @return point
 */
inline static std::tuple<bool,Point> getIntersect(Point p1, Point q1, Point p2, Point q2)
{
  Point output;
  bool on_wall = false;
  output.x = -1000;
  output.y = -1000;

  // first filter out lines that do not intersect
  if (((q1.x-p1.x)==0 && (q2.x-p2.x)==0) or ((q1.y-p1.y)==0 && (q2.y-p2.y)==0))
  {
    //points are not intersecting, so we set extreme values to make sure it's not picked as a solution
    output.x = -1000;
    output.y = -1000;
    on_wall = false;
  }
  // case one: two lines that are not completely vertical
  else if ( (q1.x-p1.x)!=0 && (q2.x-p2.x)!=0)
  {
  // make functions in the form of y = a*x + b
  float a_1 = (q1.y-p1.y)/(q1.x-p1.x);
  float b_1 = q1.y-a_1*q1.x;
  
  float a_2 = (q2.y-p2.y)/(q2.x-p2.x);
  float b_2 = q2.y-a_2*q2.x;
  
  output.x = (b_2-b_1)/(a_1-a_2);
  output.y = a_1*output.x + b_1; 

  }
  // case two: two lines that are not completely horizontal
  else if ((q1.y-p1.y)!=0 && (q2.y-p2.y)!=0)
  {
      // make functions in the form of x = a*y + b
    float a_1 = (q1.x-p1.x)/(q1.y-p1.y);
    float b_1 = q1.x-a_1*q1.y;
    
    float a_2 = (q2.x-p2.x)/(q2.y-p2.y);
    float b_2 = q2.x-a_2*q2.y;

    output.y = (b_2-b_1)/(a_1-a_2);
    output.x = a_1*output.y + b_1;
  }

  // case three: first line is horizontal, second line is vertical
  else if ((q1.y-p1.y)==0 && (q2.x-p2.x) ==0 )
  {
    output.y = q1.y;
    output.x = q2.x;
  }
  // case four: first line is vertical, second line is horizontal
  else if ((q1.x-p1.x)==0 && (q2.y-p2.y) ==0 )
  {
    output.y = q2.y;
    output.x = q1.x;
  }
  
  if(onSegment(p2,output,q2) && onSegment(p1,output,q1))
  {
    on_wall = true;
  }
  

  return std::make_tuple(on_wall,output);
}

inline static float getDistance(Point p1, Point p2)
{
  return (std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)));
}




/**
 * Get current date/time, format is YYYY-MM-DD-hh:mm:ss
 * Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
 * for more information about date/time format
 *
 * @return A character string with the current date and time.
 */
inline static const std::string currentDateTime()
{
  time_t now = time(0); // Read in the time
  struct tm tstruct;
  char buf[80]; // Buffer#include <fstream>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
  tstruct = *localtime(&now);

  // Put the time on a string using the buffer
  strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);
  return buf;
}

//helper struct used for serialization with cereal


CEREAL_CLASS_VERSION(SomeData, 1);

struct MyType
{
  // int x;
  // double y;
  Gasdata gas_obj;

  template <class Archive>
  void serialize( Archive & ar, std::uint32_t const version )
  {
    // ar( x, y );
    // ar( s );
    ar(gas_obj);
  }
};


inline static void save_gas_object(Gasdata &gas_obj,std::string env_dir)
{
  std::string filename = "conf/environments/"+env_dir +"/gas_data.bin"; 
  std::ofstream os(filename, std::ios::binary);

  {
    cereal::BinaryOutputArchive ar(os);
    MyType m;
    m.gas_obj = gas_obj;
    ar( m );
  }
}

inline static Gasdata load_gas_object(std::string env_dir)
{
  std::string filename = "conf/environments/"+env_dir +"/gas_data.bin"; 
  std::ifstream os(filename, std::ios::binary);
  cereal::BinaryInputArchive iarchive(os);
  MyType output;
  iarchive(output);
  return output.gas_obj;

}




#endif /*AUXILIARY_H*/

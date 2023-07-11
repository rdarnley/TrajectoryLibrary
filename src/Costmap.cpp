#include "TrajectoryLibrary/Costmap.h"

/// @brief Constructor
Costmap::Costmap()
{}


Costmap::Costmap( double minXArg, double minYArg, double resolutionArg, int widthArg, int heightArg )
    :   min_x{minXArg},
        min_y{minYArg},
        resolution{resolutionArg},
        width{widthArg},
        height{heightArg}
{
    max_x = min_x + width * resolution;
    max_y = min_y + height * resolution;
}


std::pair<int, int> Costmap::indexToRC( int index ) const
{
    int c = index % width;
    int r = ( index - c ) / width;
    return std::pair<int, int>( r, c );
}

bool Costmap::inMap( double x, double y ) const 
{
    return (min_x < x) && (x < (min_x + width * resolution)) &&
            (min_y < y) && (y < (min_y + height * resolution));
}

int Costmap::xToCol( double x ) const
{
    return floor(( x - min_x ) / resolution);
}

double Costmap::colToX( int col ) const
{
    return min_x + col * resolution;
}

int Costmap::yToRow( double y ) const
{
    return floor(( y - min_y ) / resolution);
}

double Costmap::rowToY( int row ) const
{
    return min_y + row * resolution;
}

uint8_t Costmap::get( int r, int c ) const
{
    if (( r < 0 ) || ( height <= r ) || ( c < 0 ) || ( width <= c )){
        std::cerr << "[Costmap] Get Function Called Outside Map Bounds (row, col) : " << r << " " << c << std::endl;
        return 0;
    }

    return data[width * r + c];
}

uint8_t Costmap::get( double x, double y ) const
{
    return get( yToRow( y ), xToCol( x ) );
}


// ///// Load Costmap To/From Files

// bool Costmap::fromFile( const std::string& filename, double resolution ) {
//   if( boost::algorithm::ends_with( filename, "pgm" ) ){
//     return fromPgm( filename, resolution );
//   } else {
//     try {
//       std::stringstream error_msg;
//       error_msg << "[Costmap] Erroneous file suffix (\"" << filename << "\") in from_file";
//       throw std::runtime_error( error_msg.str() );
//     } catch ( const char* msg ){
//       std::cerr << msg << std::endl;
//     }
//   }
//   return false;
// }


// bool Costmap::fromPgm( const std::string& filename, double resolutionArg ) {
//   data.clear();

//   // Read in the header information
//   std::ifstream infile; // Input stream class
//   infile.open( filename );
  
//   std::string identifier;
//   std::getline( infile, identifier );
//   std::string comment;
//   std::getline( infile, comment );
  
//   if ( comment.at(0) == '#' ) { // Ignore this line
//     infile >> width >> height;
//   } else {
//     char chars[20];
//     comment.copy( chars, comment.length() ); // copies comment std::string into char* chars
//     char* pch = strtok( chars, " " );
//     width = std::strtol( pch, nullptr, 10 );
//     pch = strtok( chars, " " );
//     height = std::strtol( pch, nullptr, 10 );
//   }
    
//   unsigned int max_val;
//   infile >> max_val;
  
//   assert( max_val == 255 );
  
//   int size = width*height;
//   std::vector<uint8_t> imported_data;
  
//   std::string line;
//   std::getline( infile, line );
//   for (int i = 0; i < size; i++) {
//     std::getline( infile, line );
//     int pixel = std::stol( line, nullptr, 10 );
//     imported_data.push_back( (uint8_t) pixel );
//   }
  
//   // Now flip imported pixels s.t. positive y is up in the image.
//   data.clear();
//   for (int c = 0; c < width; c++) {
//     for (int data_r = 0; data_r < height; data_r++) {
//       int imported_r = height - data_r - 1;
//       int imported_index = height*c + imported_r;
//       data.push_back( imported_data[imported_index] );
//     }
//   }
    
//   // Set rest of the CostMap's member variables.
//   resolution = resolutionArg;
  
//   double half_total_width = 0.5 * width * resolution;
//   double half_total_height = 0.5 * height * resolution;
  
//   min_x = -half_total_width; // By default, center impirted CostMaps.
//   min_y = -half_total_height;
//   max_x = min_x + width*resolution;
//   max_y = min_y + height*resolution;
	
//   return true;
// }




// void Costmap::toFile( const std::string& filename )const {
//   if ( boost::algorithm::ends_with( filename, "pgm" ) ){
//     return toPgm( filename );
//   } else {
//     try {
//       std::stringstream error_msg;
//       error_msg << "[Costmap] Erroneous file suffix (\"" << filename << "\") in to_file";
//       throw std::runtime_error( error_msg.str() );
//     } catch ( const char* msg ){
//       std::cerr << msg << std::endl;
//     }
//   }
//   return;
// }

// void Costmap::toPgm( const std::string& filename )const {
//   std::ofstream outfile; // output file stream
//   outfile.open( filename ); // ASCII output the entire time
  
//   // P2 indicates ASCII form of the PGM image data
//   outfile << "P2\n";
//   outfile << width << " " << height << "\n"; // width and height output as ASCII numbers
//   outfile << "255\n";

//   for ( const uint8_t & pixel : data ) {
//     outfile << ((int) pixel) << "\n";
//   }
  
//   outfile.close();
//   return;
// }
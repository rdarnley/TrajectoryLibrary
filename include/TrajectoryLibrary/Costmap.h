#ifndef COSTMAP_H
#define COSTMAP_H

#include <thread>
#include <condition_variable>
#include <iostream>

#include <vector>
#include <math.h>

class Costmap {
    public:

        /// @brief Empty Constructor
        Costmap();

        /// @brief Default Constructor
        Costmap( double minXArg, double minYArg,
                 double resolutionArg, int widthArg,
                 int heightArg );


        // /// @brief Default Destructor
        // ~Costmap();

        //// Utility Functions
        std::pair<int, int> indexToRC( int index ) const;
        bool inMap( double x, double y ) const;
        int xToCol( double x ) const;
        double colToX( int col ) const;
        int yToRow( double y ) const;
        double rowToY( int row ) const;

        uint8_t get( int r, int c ) const;
        uint8_t get( double x, double y ) const;

        // ///// Load Costmap To/From Files
        // bool fromFile( const std::string& filename, double resolution );
        // bool fromPgm( const std::string& filename, double resolution );
        // bool toFile( const std::string& filename ) const;
        // bool toPgm( const std::string& filename ) const;


        std::vector<uint8_t> data;

        double resolution;
        double min_x;
        double min_y;
        double max_x;
        double max_y;

        int width;
        int height;

    private:

        // std::vector<uint8_t> data;

        // double resolution;
        // double min_x;
        // double min_y;
        // double max_x;
        // double max_y;

        // int width;
        // int height;

        // std::ostream& operator<<( std::ostream& out, const Costmap& other);
};

#endif




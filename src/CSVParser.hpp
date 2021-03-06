#ifndef __CSVPARSER_HPP__
#define __CSVPARSER_HPP__

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "Common.hpp"

using std::string;
using std::vector;
using Eigen::MatrixXd;

#define CSV_EXCEPTION_FILE_NOT_EXIST(f) \
    {\
        std::stringstream f##_ss;\
        f##_ss << "Input CSV file " \
               << f << " does not exist.";\
        BOOST_THROW_EXCEPTION( CSVFileNotExist() << wz::ExceptionInfoString(f##_ss.str()) );\
    }

namespace csv
{

struct CSVException    : virtual wz::exception_base { };
struct CSVFileNotExist : virtual CSVException{ };

typedef double Real_t;

class CSVParser
{
public:
    typedef struct 
    {
        string name;
        Real_t east;
        Real_t north;
        Real_t height;
        Real_t roll;
        Real_t pitch;
        Real_t yaw;
    } Entry_t;

    typedef enum
    {
        idxName = 0,
        idxEast,
        idxNorth,
        idxHeight,
        idxRoll,
        idxPitch,
        idxYaw
    } EntryIdx_t;

public:
    CSVParser();
    ~CSVParser();

    static void parse( const string inFn, vector<Entry_t>& table, Real_t shiftEast = 0.0, Real_t shiftNorth = 0.0, int omit = 0 );

    static void convert_string_to_entry( const vector<string>& line, Entry_t& entry );
    static void show_single_entry( const Entry_t& entry );
    static void show_table( const vector<Entry_t>& table );

    static void convert_data_to_Eigen(const vector<Entry_t>& table, MatrixXd& m);
};

typedef vector<CSVParser::Entry_t> Table_t;

void test_read_csv(const string& inFn);

}
#endif // __CSVPARSER_HPP__
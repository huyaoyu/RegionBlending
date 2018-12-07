
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include "CSVParser.hpp"

using namespace std;
using namespace csv;

CSVParser::CSVParser()
{

}

CSVParser::~CSVParser()
{

}

void CSVParser::convert_string_to_entry( const vector<string>& line, Entry_t& entry )
{
    entry.name = line[ idxName ];
    // C++11.
    entry.east   = stod( line[ idxEast ] );
    entry.north  = stod( line[ idxNorth ] );
    entry.height = stod( line[ idxHeight ] );

    if ( line.size() > 4 )
    {
        entry.roll   = stod( line[ idxRoll ] );
        entry.pitch  = stod( line[ idxPitch ] );
        entry.yaw    = stod( line[ idxYaw ] );
    }
    else
    {
        entry.roll   = 0.0;
        entry.pitch  = 0.0;
        entry.yaw    = 0.0;
    }
}

void CSVParser::show_single_entry( const Entry_t& entry )
{
    cout.precision(2);

    cout << entry.name << ", "
         << scientific << showpos
         << entry.east << ", "
         << entry.north << ", "
         << entry.height << ", "
         << entry.roll << ", "
         << entry.pitch << ", "
         << entry.yaw << noshowpos << endl;
}

void CSVParser::show_table( const vector<Entry_t>& table )
{
    int count = 1;

    for ( auto iterTable = table.begin(); iterTable != table.end(); ++iterTable )
    {
        cout << noshowpos << count << ": ";

        show_single_entry( *iterTable );

        count++;
    }
}

void CSVParser::convert_data_to_Eigen(const vector<Entry_t>& table, MatrixXd& m)
{
    // Get the number of rows in table.
    int rows = table.size();

    // Allocate memory for m.
    m = MatrixXd(rows, 6);

    Entry_t tempEntry;

    int i = 0;
    for( auto iter = table.begin(); iter != table.end(); ++i, ++iter )
    {
        m(i, 0) = (*iter).east;
        m(i, 1) = (*iter).north;
        m(i, 2) = (*iter).height;
        m(i, 3) = (*iter).roll;
        m(i, 4) = (*iter).pitch;
        m(i, 5) = (*iter).yaw;
    }
}

void CSVParser::parse( string inFn, vector<Entry_t>& table,
    Real_t shiftEast, Real_t shiftNorth, int omit )
{
    string line;

    ifstream infile( inFn );
    // Omit lines.
    while ( omit != 0 )
    {
        getline(infile, line);
        omit--;
    }
    
    boost::tokenizer<boost::escaped_list_separator<char> >::iterator iterT;
    vector<string> vec;
    string tempStr;
    Entry_t tempEntry;

    while ( getline( infile, line ) )
    {
        boost::tokenizer<boost::escaped_list_separator<char> > tk(
            line, boost::escaped_list_separator<char>('\\', ',', '\"'));

        for (iterT = tk.begin(); iterT != tk.end(); ++iterT) 
        {
            tempStr = *iterT;

            boost::trim( tempStr );

            vec.push_back(tempStr);
        }

        convert_string_to_entry( vec, tempEntry );

        tempEntry.east  += shiftEast;
        tempEntry.north += shiftNorth;

        table.push_back( tempEntry );

        vec.clear();
    }
}

void csv::test_read_csv(const string& inFn)
{
    // Create a CSVParser object.
    CSVParser csvParser;
    vector<CSVParser::Entry_t> table;

    csvParser.parse( inFn, table );
    CSVParser::show_table( table );

    MatrixXd m;
    CSVParser::convert_data_to_Eigen( table, m );

    cout << "The Eigen matrix is :" << endl;
    cout << m << endl;
}

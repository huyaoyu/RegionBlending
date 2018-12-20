#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include <exception>
#include <iostream>
#include <sstream>
#include <string>

#include <boost/exception/all.hpp>
#include <boost/shared_ptr.hpp>

#define EXCEPTION_ARG_OUT_OF_RANGE(v, minV, maxV) \
    {\
        std::stringstream v##_ss;\
        v##_ss << "Argument out of range, " \
               << #v << " = " << v \
               << ", [" << minV << ", " << maxV << "]. "\
               << "Value not changed.";\
        BOOST_THROW_EXCEPTION( argument_out_of_range() << ExceptionInfoString(v##_ss.str()) );\
    }

#define EXCEPTION_ARG_NULL(v) \
    {\
        std::stringstream v##_ss;\
        v##_ss << "Argument " \
               << #v << " is NULL.";\
        BOOST_THROW_EXCEPTION( argument_null() << ExceptionInfoString(v##_ss.str()) );\
    }

#define EXCEPTION_BASE(s) \
    {\
        BOOST_THROW_EXCEPTION( exception_base() << ExceptionInfoString(s) );\
    }

namespace wz
{

struct exception_base        : virtual std::exception, virtual boost::exception { };
struct bad_argument          : virtual exception_base { };
struct argument_out_of_range : virtual bad_argument { };
struct argument_null         : virtual bad_argument { };
struct third_party           : virtual exception_base { };

typedef boost::error_info<struct tag_info_string, std::string> ExceptionInfoString;

}


#endif // __COMMON_HPP__
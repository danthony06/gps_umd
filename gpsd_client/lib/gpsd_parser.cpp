#include "gpsd_parser.hpp"
#include "gpsd_parser_v11.hpp"

namespace gpsd_client
{
GpsdParser::GpsdParser()
{
  parse = parse_v11;
}
}
#include "ocra_debug.h"

#include <fstream>

namespace ocra_debug
{
  std::ofstream ocra_dbg_1("ocra_debug_1.txt");
  std::ofstream ocra_dbg_2("ocra_debug_2.txt");
  std::ofstream ocra_dbg_3("ocra_debug_3.txt");
  std::ofstream ocra_dbg_4("ocra_debug_4.txt");
  std::ofstream ocra_dbg_5("ocra_debug_5.txt");
}

// cmake:sourcegroup=Utils

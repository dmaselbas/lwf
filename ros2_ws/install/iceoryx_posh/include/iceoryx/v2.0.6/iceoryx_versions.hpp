#ifndef __ICEORYXVERSIONS__
#define __ICEORYXVERSIONS__

#define ICEORYX_VERSION_MAJOR    2
#define ICEORYX_VERSION_MINOR    0
#define ICEORYX_VERSION_PATCH    6
#define ICEORYX_VERSION_TWEAK    0

#define ICEORYX_LATEST_RELEASE_VERSION    "2.0.6"
#define ICEORYX_BUILDDATE                 "2024-09-08T05:38:47Z"
#define ICEORYX_SHA1                      "ebdf38728a29c6a8ee172efc440b8e2f4415ff7a"

#include "iceoryx_posh/internal/log/posh_logging.hpp"

#define ICEORYX_PRINT_BUILDINFO()     iox::LogInfo() << "Built: " << ICEORYX_BUILDDATE;


#endif

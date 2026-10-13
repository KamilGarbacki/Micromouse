// Short Serial name of a Status code, shared by the FAULT report and the
// run log printer (one flash string table instead of two).
#pragma once

#include "hal/log.h"
#include "util/status.h"

namespace micromouse {

const __FlashStringHelper* statusName(Status status);

}  // namespace micromouse

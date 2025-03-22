#ifndef IMM_CS_Parallel_H
#define IMM_CS_Parallel_H

#include "combined_models/IMM.h"
#include "basic_models/CS_KF.h"

class IMM_CS_Parallel : public IMM {
public:
    IMM_CS_Parallel();
      ~IMM_CS_Parallel() = default;
};

#endif // IMM_CS_Parallel

#include "ExtrinsicErrorTerm/ExtrinsicErrorTerm.h"

int main()
{
    ExtrinsicErrorTerm::Ptr pExtrinsicErrorTerm;
    pExtrinsicErrorTerm = std::shared_ptr<ExtrinsicErrorTerm>(new ExtrinsicErrorTerm());
    pExtrinsicErrorTerm->readPose(".");
    pExtrinsicErrorTerm->readSonarWaveData(".");
    bool useIcpAlign = false;
    bool useCeresAlign = true;
    if (useIcpAlign)
    {
        pExtrinsicErrorTerm->buildMap();
        pExtrinsicErrorTerm->align();
    }
    else if (useCeresAlign)
    {
        pExtrinsicErrorTerm->ceresAlign();
    }
    return 0;
}
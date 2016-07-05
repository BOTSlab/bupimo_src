#include "TotalHomingAlg.h"
#include "SingleConfig.h"

// For the various homing alg's
//#include "HissAlg.h"
//#include "FBAlg.h"
#include "TwoStepHomingAlg.h"
//#include "OpticFlowHomingAlg.h"
#include "DIDCompass.h"
//#include "FlowHomingAlg.h"
#include "WarpingAlg.h"

TotalHomingAlg* TotalHomingAlg::createFromConfig(int width, int height) {
    SingleConfig *config = SingleConfig::getInstance();

    string algName = config->getString("TotalHomingAlg.algName", "Warping");
    
/*    if (algName == "Hiss") {
        return new HissAlg(width, height);
    } else */ if (algName == "Warping") {
        return new WarpingAlg(width, height);
/*    } if (algName == "FB") {
        return new FBAlg(width, height);
    } else if (algName == "DID_OpticFlow") {
        DIDCompass *didCompass = new DIDCompass(width, height);
        OpticFlowHomingAlg *opticFlow = new OpticFlowHomingAlg(width, height);
        return new TwoStepHomingAlg(width, height, didCompass, opticFlow);
    } else if (algName == "OpticFlow") {
		int camRange = config->getInt("FlowHomingAlg.cameraRange", 26);
		return new FlowHomingAlg(width, height, (camRange/180.0) * M_PI); 
*/    } else {
        assert(false);
        return NULL;
    }
}

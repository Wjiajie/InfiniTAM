// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "Interface/ITMSurfelVisualisationEngine.h"
#include "../../Utils/Configuration.h"

namespace ITMLib
{
  /**
   * \brief An instantiation of this struct can be used to construct surfel visualisation engines.
   */
  template <typename TSurfel>
  struct ITMSurfelVisualizationEngineFactory
  {
    //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

    /**
     * \brief Makes a surfel visualisation engine.
     *
     * \param deviceType  The device on which the surfel visualisation engine should operate.
     * \return            The surfel visualisation engine.
     */
    static ITMSurfelVisualisationEngine<TSurfel> *make_surfel_visualisation_engine(MemoryDeviceType deviceType);
  };
}

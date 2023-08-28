/**
 * \file EnvireOdePhysicsPlugins.hpp
 * \author Malte Langosz
 * \brief Plugin class to load physics representation based on envire items
 *
 */

#pragma once

// TODO: check and clean the header includes

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/Vector.h>

#include <mars_interfaces/sim/SimulatorInterface.h>

#include <lib_manager/LibInterface.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_base_types/sensors/CameraSensor.hpp>
#include <envire_base_types/sensors/RaySensor.hpp>

#include <iostream>

namespace mars
{
    namespace envire_sensors
    {
        // move the typedef to separate file
        class EnvireSensorsPlugins : public lib_manager::LibInterface,
                                    public envire::core::GraphEventDispatcher,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::sensors::CameraSensor>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::sensors::RaySensor>>
        {

        public:
            EnvireSensorsPlugins(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            virtual ~EnvireSensorsPlugins();

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string("envire_mars_sensors");
            }

            CREATE_MODULE_INFO();

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::sensors::CameraSensor>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::sensors::RaySensor>>& e) override;

        private:
            interfaces::SimulatorInterface *sim;

            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);

            void createSensor(configmaps::ConfigMap &config, const std::string &frameId);
        };

    } // end of namespace envire_sensors
} // end of namespace mars

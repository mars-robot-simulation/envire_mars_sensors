/**
 * \file EnvireSensorsPlugins.hpp
 * \author Malte Langosz
 * \brief Plugin class to load sensor representations based on envire items
 *
 */

#pragma once


#include <lib_manager/LibInterface.hpp>

#include <mars_interfaces/sim/SimulatorInterface.h>

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_types/sensors/CameraSensor.hpp>
#include <envire_types/sensors/RaySensor.hpp>
#include <envire_types/sensors/RotatingRaySensor.hpp>
#include <envire_types/sensors/Joint6DOFSensor.hpp>

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/Vector.h>

namespace mars
{
    namespace envire_sensors
    {
        // move the typedef to separate file
        class EnvireSensorsPlugins : public lib_manager::LibInterface,
                                    public envire::core::GraphEventDispatcher,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::sensors::CameraSensor>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::sensors::RaySensor>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::sensors::RotatingRaySensor>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::sensors::Joint6DOFSensor>>
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
                return std::string{"envire_mars_sensors"};
            }

            CREATE_MODULE_INFO();

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::sensors::CameraSensor>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::sensors::RaySensor>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::sensors::RotatingRaySensor>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::sensors::Joint6DOFSensor>>& e) override;

        private:
            interfaces::SimulatorInterface *sim;

            // TODO: Move to central location
            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);

            void createSensor(configmaps::ConfigMap &config, const std::string &frameId);
        };

    } // end of namespace envire_sensors
} // end of namespace mars
